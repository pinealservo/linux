/*
 * Marvell 88E6xxx Switch PTP support
 *
 * Copyright (c) 2008 Marvell Semiconductor
 *
 * Copyright (c) 2017 National Instruments
 *      Erik Hons <erik.hons@ni.com>
 *      Brandon Streiff <brandon.streiff@ni.com>
 *      Dane Wagner <dane.wagner@ni.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "chip.h"
#include "global2.h"
#include "ptp.h"

#define TAI_EVENT_WORK_INTERVAL msecs_to_jiffies(100)

#define NS_PER_CYC 8
#define FRAC_SHIFT 28
#define CC_WIDTH 32
#define CC_MULT (NS_PER_CYC << FRAC_SHIFT)
#define PEROUT_DELAY_CYCLES 3
#define TRIGGER_LOCK_DELAY (50000000ULL)

static int mv88e6xxx_tai_read(struct mv88e6xxx_chip *chip, int addr,
			      u16 *data, int len)
{
	if (!chip->info->ops->avb_ops->tai_read)
		return -EOPNOTSUPP;

	return chip->info->ops->avb_ops->tai_read(chip, addr, data, len);
}

static int mv88e6xxx_tai_write(struct mv88e6xxx_chip *chip, int addr, u16 data)
{
	if (!chip->info->ops->avb_ops->tai_write)
		return -EOPNOTSUPP;

	return chip->info->ops->avb_ops->tai_write(chip, addr, data);
}

static u64 mv88e6xxx_ptp_clock_read(const struct cyclecounter *cc)
{
	struct mv88e6xxx_chip *chip =
		container_of(cc, struct mv88e6xxx_chip, tstamp_cc);
	int err;
	u16 phc_time[2];

	err = mv88e6xxx_tai_read(chip, MV88E6XXX_TAI_TIME_LO, phc_time,
				 ARRAY_SIZE(phc_time));
	if (err)
		return 0;
	else
		return ((u32)phc_time[1] << 16) | phc_time[0];
}

static void mv88e6xxx_calc_local_period(u32 period, long scaled_ppm, u32 *cycles, s32 *error)
{
	u64 local_diff, local_period;
	u32 sppm, period_cyc;
	s32 scaled_picos;
	s16 picos, subpicos;
	int neg_adj = 0;

	/* Separate magnitude from direction */
	if (scaled_ppm < 0) {
		neg_adj = 1;
		sppm = -scaled_ppm;
	} else {
		sppm = scaled_ppm;
	}

	/* Calculate local period difference from nominal */
	local_diff = (u64)period * (u64)sppm;
	local_diff = div_u64(local_diff, 1000000);

	/* Adjust local period by difference to match nominal */
	local_period = (u64)period << 16;
	local_period = neg_adj ?
		local_period + local_diff :
		local_period - local_diff;

	/* Divide local period into a cycle count and per-period error */
	period_cyc = (local_period / NS_PER_CYC) >> 16;
	local_diff = (period_cyc * NS_PER_CYC) << 16;
	scaled_picos = ((local_period - local_diff) * 1000) >> 8;

	/* Adjust per-period error to be less than half a cycle */
	picos = scaled_picos >> 8;
	subpicos = scaled_picos & 0xFF;
	if (picos > (NS_PER_CYC / 2)) {
		period_cyc += 1;
		picos -= (NS_PER_CYC * 1000);
		scaled_picos = (picos << 8) | subpicos;
	}

	*cycles = period_cyc;
	*error = scaled_picos;
}

static int mv88e6xxx_disable_trig(struct mv88e6xxx_chip *chip)
{
	int err;
	u16 global_config;

	chip->trig_config = 0;
	global_config = (chip->evcap_config | chip->trig_config);
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_CFG, global_config);

	return err;
}

static int mv88e6xxx_update_periodic_trig(struct mv88e6xxx_chip *chip,
					  u32 cycles, s32 scaled_picos)
{
	int err = 0;
	int update = 0;
	u16 picos, subpicos;


	/* Split scaled_picos into separate sign/magnitude picos value
	 * and unsigned subpicos value
	 */
	if (scaled_picos < 0) {
		picos = BIT(15) | ((-scaled_picos) >> 8);
		subpicos = (-scaled_picos) & 0xFF;
	} else {
		picos = scaled_picos >> 8;
		subpicos = scaled_picos & 0xFF;
	}


	if (cycles != chip->perout.period_cycles) {
		err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_LO,
					  cycles & 0xffff);
		if (err)
			return err;

		err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_HI,
					  cycles >> 16);
		if (err)
			return err;

		chip->perout.period_cycles = cycles;
		update = 1;
	}

	if ((scaled_picos & 0xFFFFFF00) != (chip->perout.scaled_picos & 0xFFFFFF00)) {
		err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CLOCK_COMP,
					  picos);
		if (err)
			return err;

		update = 1;
	}

	/* In block update mode, we must write the subpicos register if any
	 * other value changed in order to do an atomic update.
	 */
	if (update ||
	    ((scaled_picos & 0xFF) != (chip->perout.scaled_picos & 0xFF))) {

		err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CFG,
					  subpicos & 0xFF);

		chip->perout.scaled_picos = scaled_picos;
	}



	return err;
}

static int mv88e6xxx_config_periodic_trig(struct mv88e6xxx_chip *chip,
					  u32 cycles, s32 scaled_picos,
					  u32 start)
{
	int err;
	u16 global_config, picos, subpicos;

	/* Split scaled_picos into separate sign/magnitude picos value
	 * and unsigned subpicos value
	 */
	if (scaled_picos < 0) {
		picos = BIT(15) | ((-scaled_picos) >> 8);
		subpicos = (-scaled_picos) & 0xFF;
	} else {
		picos = scaled_picos >> 8;
		subpicos = scaled_picos & 0xFF;
	}

	/* Program start time as a cycle counter value */
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_LO,
				  start & 0xffff);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_HI,
				  start >> 16);
	if (err)
		return err;

	/* Program period in cycles */
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_LO,
				  cycles & 0xffff);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_HI,
				  cycles >> 16);
	if (err)
		return err;

	/* Program period error in picoseconds per period */
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CLOCK_COMP,
				  picos);
	if (err)
		return err;

	/* Program period error fractional picoseconds */
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CFG,
				  subpicos & 0xFF);
	if (err)
		return err;

	/* Start the periodic trigger mechanism */
	chip->trig_config = MV88E6XXX_TAI_CFG_BLOCK_UPDATE |
		MV88E6XXX_TAI_CFG_TRIG_ENABLE;
	global_config = (chip->evcap_config | chip->trig_config);
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_CFG, global_config);

	/* Cache the period measurement to avoid redundant writes */
	chip->perout.period_cycles = cycles;
	chip->perout.scaled_picos = scaled_picos;

	return err;
}

/* mv88e6xxx_config_eventcap - configure TAI event capture
 * @event: PTP_CLOCK_PPS (internal) or PTP_CLOCK_EXTTS (external)
 * @rising: zero for falling-edge trigger, else rising-edge trigger
 *
 * This will also reset the capture sequence counter.
 */
static int mv88e6xxx_config_eventcap(struct mv88e6xxx_chip *chip, int event,
				     int rising)
{
	u16 global_config;
	u16 cap_config;
	int err;

	chip->evcap_config = MV88E6XXX_TAI_CFG_CAP_OVERWRITE |
			     MV88E6XXX_TAI_CFG_CAP_CTR_START;
	if (!rising)
		chip->evcap_config |= MV88E6XXX_TAI_CFG_EVREQ_FALLING;

	global_config = (chip->evcap_config | chip->trig_config);
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_CFG, global_config);
	if (err)
		return err;

	if (event == PTP_CLOCK_PPS) {
		cap_config = MV88E6XXX_TAI_EVENT_STATUS_CAP_TRIG;
	} else if (event == PTP_CLOCK_EXTTS) {
		/* if STATUS_CAP_TRIG is unset we capture PTP_EVREQ events */
		cap_config = 0;
	} else {
		return -EINVAL;
	}

	/* Write the capture config; this also clears the capture counter */
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_EVENT_STATUS,
				  cap_config);

	return err;
}

static void mv88e6xxx_tai_event_work(struct work_struct *ugly)
{
	struct delayed_work *dw = to_delayed_work(ugly);
	struct mv88e6xxx_chip *chip =
		container_of(dw, struct mv88e6xxx_chip, tai_event_work);
	u16 ev_status[4];
	int err;

	mutex_lock(&chip->reg_lock);

	err = mv88e6xxx_tai_read(chip, MV88E6XXX_TAI_EVENT_STATUS,
				 ev_status, ARRAY_SIZE(ev_status));
	if (err) {
		mutex_unlock(&chip->reg_lock);
		return;
	}

	if (ev_status[0] & MV88E6XXX_TAI_EVENT_STATUS_ERROR)
		dev_warn(chip->dev, "missed event capture\n");

	if (ev_status[0] & MV88E6XXX_TAI_EVENT_STATUS_VALID) {
		struct ptp_clock_event ev;
		u32 raw_ts = ((u32)ev_status[2] << 16) | ev_status[1];

		/* Clear the valid bit so the next timestamp can come in */
		ev_status[0] &= ~MV88E6XXX_TAI_EVENT_STATUS_VALID;
		err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_EVENT_STATUS,
					  ev_status[0]);

		if (ev_status[0] & MV88E6XXX_TAI_EVENT_STATUS_CAP_TRIG) {
			/* TAI is configured to timestamp internal events.
			 * This will be a PPS event.
			 */
			ev.type = PTP_CLOCK_PPS;
		} else {
			/* Otherwise this is an external timestamp */
			ev.type = PTP_CLOCK_EXTTS;
		}
		/* We only have one timestamping channel. */
		ev.index = 0;
		ev.timestamp = timecounter_cyc2time(&chip->tstamp_tc, raw_ts);

		ptp_clock_event(chip->ptp_clock, &ev);
	}

	mutex_unlock(&chip->reg_lock);

	schedule_delayed_work(&chip->tai_event_work, TAI_EVENT_WORK_INTERVAL);
}

static int mv88e6xxx_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	u64 adj;
	u32 mult, diff, sppm, period_cyc;
	s32 scaled_picos;
	int neg_adj = 0;
	int err = 0;
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);

	if (scaled_ppm < 0) {
		neg_adj = 1;
		sppm = -scaled_ppm;
	} else {
		sppm = scaled_ppm;
	}

	if (scaled_ppm == chip->perout.scaled_ppm)
		return 0;

	mult = CC_MULT;
	adj = mult;
	adj = adj * sppm;
	adj = div_u64(adj, 1000000);
	diff = (adj >> 16) + ((adj >> 15) & 1);

	mult = neg_adj ? CC_MULT - diff : CC_MULT + diff;

	if (chip->perout.nominal_period) {
		mv88e6xxx_calc_local_period(chip->perout.nominal_period,
					    scaled_ppm, &period_cyc, &scaled_picos);
	} else {
		period_cyc = 0;
	}

	mutex_lock(&chip->reg_lock);
	timecounter_read(&chip->tstamp_tc);
	chip->tstamp_cc.mult = mult;
	chip->perout.scaled_ppm = scaled_ppm;
	if (period_cyc)
		err = mv88e6xxx_update_periodic_trig(chip, period_cyc, scaled_picos);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);

	mutex_lock(&chip->reg_lock);
	timecounter_adjtime(&chip->tstamp_tc, delta);
	mutex_unlock(&chip->reg_lock);

	return 0;
}

static int mv88e6xxx_ptp_gettime(struct ptp_clock_info *ptp,
				 struct timespec64 *ts)
{
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);
	u64 ns;

	mutex_lock(&chip->reg_lock);
	ns = timecounter_read(&chip->tstamp_tc);
	chip->last_overflow_check = jiffies;
	mutex_unlock(&chip->reg_lock);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int mv88e6xxx_ptp_settime(struct ptp_clock_info *ptp,
				 const struct timespec64 *ts)
{
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);
	u64 ns;

	ns = timespec64_to_ns(ts);

	mutex_lock(&chip->reg_lock);
	timecounter_init(&chip->tstamp_tc, &chip->tstamp_cc, ns);
	mutex_unlock(&chip->reg_lock);

	return 0;
}

static int mv88e6xxx_ptp_enable_extts(struct mv88e6xxx_chip *chip,
				      struct ptp_clock_request *rq, int on)
{
	int rising = (rq->extts.flags & PTP_RISING_EDGE);
	int pin;
	int err;

	pin = ptp_find_pin(chip->ptp_clock, PTP_PF_EXTTS, rq->extts.index);

	if (pin < 0)
		return -EBUSY;

	mutex_lock(&chip->reg_lock);

	if (on) {
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_EVREQ,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_IN);
		if (err)
			goto out;

		schedule_delayed_work(&chip->tai_event_work,
				      TAI_EVENT_WORK_INTERVAL);

		err = mv88e6xxx_config_eventcap(chip, PTP_CLOCK_EXTTS,
						rising);
	} else {
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_GPIO,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_IN);

		cancel_delayed_work_sync(&chip->tai_event_work);
	}

out:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static void mv88e6xxx_ptp_trigger_lock(struct mv88e6xxx_chip *chip)
{
	u64 now_ns, edge_ns, frac_ns, wait_cycles, cycle, periods;
	u32 mult, shift;
	u16 global_config, status;
	int err, corr;

	/* Nothing to do if we don't have a period for output */
	if (chip->perout.nominal_period == 0)
		return;

	mutex_lock(&chip->reg_lock);

	/* Read the status value for the last lock attempt */
	err = mv88e6xxx_tai_read(chip, MV88E6XXX_TAI_CFG, &status, 1);
	if (err)
		goto out;

	/* Don't set a new trigger lock if the previous one hasn't completed */
	if (status & MV88E6XXX_TAI_CFG_TRIG_LOCK) {
		goto out;
	}

	/* Read the previous lock status before starting a new lock */
	err = mv88e6xxx_tai_read(chip, MV88E6XXX_TAI_LOCK_STATUS, &status, 1);
	if (err)
		goto out;

	now_ns = timecounter_read(&chip->tstamp_tc);
	mult = chip->tstamp_cc.mult;
	shift = chip->tstamp_cc.shift;
	cycle = chip->tstamp_tc.cycle_last;
	frac_ns = chip->tstamp_tc.frac;

	mutex_unlock(&chip->reg_lock);

	/* Skip if we are still waiting for the first edge */
	if (now_ns < chip->perout.last_edge_time)
		return;

	/* Find the GM time of the edge to lock on */
	edge_ns = chip->perout.last_edge_time;
	periods = (now_ns - edge_ns) / chip->perout.nominal_period;
	periods += 1;
	/* Delay a number of periods that will give time to program the chip */
	periods += (TRIGGER_LOCK_DELAY / chip->perout.nominal_period);
	edge_ns += (u64)chip->perout.nominal_period * periods;

	/* Find how many local clock cycles in the future it is */
	wait_cycles = (((edge_ns - now_ns) << shift) -
		       frac_ns + (1 << (shift-1))) / mult;
	cycle = cycle + wait_cycles - PEROUT_DELAY_CYCLES;

	/* Program the trigger lock for that cycle */
	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_LO,
				  cycle & 0xffff);
	if (err)
		goto out;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_HI,
				  cycle >> 16);
	if (err)
		goto out;

	global_config = (chip->evcap_config | chip->trig_config |
			 MV88E6XXX_TAI_CFG_TRIG_LOCK |
			 MV88E6XXX_TAI_CFG_TRIG_RANGE(7));
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_CFG, global_config);
	if (err)
		goto out;

	if (status & MV88E6XXX_TAI_LOCK_STATUS_VALID) {
		corr = status & 0x7;
		if (status & 0x8)
			corr = -corr;

		chip->perout.last_edge_time = edge_ns;
		chip->perout.last_edge_cycle = cycle + PEROUT_DELAY_CYCLES;

		dev_dbg(chip->dev, "trigger lock correction: %dns\n",
			corr * NS_PER_CYC);
	} else {
		corr = 0;
	}
 out:
	mutex_unlock(&chip->reg_lock);
	return;
}

static int mv88e6xxx_ptp_enable_perout(struct mv88e6xxx_chip *chip,
				       struct ptp_clock_request *rq, int on)
{
	struct timespec ts;
	struct timecounter *tc = &chip->tstamp_tc;
	u64 period_ns, now_ns, start_ns, mask;
	u64 wait_ns, wait_cyc, frac_ns;
	u32 last_cyc, start_cyc, period_cyc, mult, shift;
	s32 scaled_picos;
	int pin;
	int err;

	pin = ptp_find_pin(chip->ptp_clock, PTP_PF_PEROUT, rq->perout.index);

	dev_dbg(chip->dev, "%s perout on pin %d\n", on ? "enable" : "disable", pin);

	if (pin < 0)
		return -EBUSY;

	if (on) {
		ts.tv_sec = rq->perout.period.sec;
		ts.tv_nsec = rq->perout.period.nsec;
		period_ns = timespec_to_ns(&ts);

		if ((period_ns / 8) > U32_MAX) {
			dev_warn(chip->dev, "period_ns too large\n");
			return -ERANGE;
		}

		ts.tv_sec = rq->perout.start.sec;
		ts.tv_nsec = rq->perout.start.nsec;
		start_ns = timespec_to_ns(&ts);

		mutex_lock(&chip->reg_lock);

		now_ns = timecounter_read(tc);
		last_cyc = tc->cycle_last;
		mask = tc->cc->mask;
		mult = chip->tstamp_cc.mult;
		shift = chip->tstamp_cc.shift;
		frac_ns = chip->tstamp_tc.frac;

		mutex_unlock(&chip->reg_lock);

		if (now_ns > start_ns) {
			dev_warn(chip->dev, "can't start in the past\n");
			return -ERANGE;
		}

		wait_ns = start_ns - now_ns;
		wait_cyc = ((wait_ns << shift) - frac_ns + (1 << (shift-1))) / mult;
		start_cyc = (last_cyc + wait_cyc) - PEROUT_DELAY_CYCLES;
		if (start_cyc > mask) {
			dev_warn(chip->dev, "too far in the future\n");
			return -ERANGE;
		}

		mv88e6xxx_calc_local_period(period_ns, chip->perout.scaled_ppm,
					    &period_cyc, &scaled_picos);

		mutex_lock(&chip->reg_lock);

		chip->perout.nominal_period = period_ns;
		chip->perout.last_edge_time = start_ns;
		chip->perout.last_edge_cycle = start_cyc + PEROUT_DELAY_CYCLES;

		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_TRIG,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_OUT);
		if (err)
			goto out;

		err = mv88e6xxx_config_periodic_trig(chip, period_cyc,
						     scaled_picos, start_cyc);
		if (err)
			goto out;

	} else {
		mutex_lock(&chip->reg_lock);

		mv88e6xxx_disable_trig(chip);
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_GPIO,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_IN);

		chip->perout.last_edge_time = 0;
		chip->perout.last_edge_cycle = 0;
		chip->perout.nominal_period = 0;
	}

out:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_ptp_enable_pps(struct mv88e6xxx_chip *chip,
				    struct ptp_clock_request *rq, int on)
{
	int pin;
	int err;

	pin = ptp_find_pin(chip->ptp_clock, PTP_PF_PEROUT, rq->extts.index);

	if (pin < 0)
		return -EBUSY;

	mutex_lock(&chip->reg_lock);

	if (on) {
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_TRIG,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_OUT);
		if (err)
			goto out;
		err = mv88e6xxx_config_periodic_trig(chip,
						     NSEC_PER_SEC/8, 0, 0);
		if (err)
			goto out;

		schedule_delayed_work(&chip->tai_event_work, 0);

		err = mv88e6xxx_config_eventcap(chip, PTP_CLOCK_PPS, 1);
	} else {
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_GPIO,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_IN);
		if (err)
			goto out;

		err = mv88e6xxx_disable_trig(chip);

		cancel_delayed_work_sync(&chip->tai_event_work);
	}

out:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_ptp_enable(struct ptp_clock_info *ptp,
				struct ptp_clock_request *rq, int on)
{
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		return mv88e6xxx_ptp_enable_extts(chip, rq, on);
	case PTP_CLK_REQ_PEROUT:
		return mv88e6xxx_ptp_enable_perout(chip, rq, on);
	case PTP_CLK_REQ_PPS:
		return mv88e6xxx_ptp_enable_pps(chip, rq, on);
	default:
		return -EOPNOTSUPP;
	}
}

static int mv88e6xxx_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
				enum ptp_pin_function func, unsigned int chan)
{
	switch (func) {
	case PTP_PF_NONE:
	case PTP_PF_EXTTS:
	case PTP_PF_PEROUT:
		break;
	case PTP_PF_PHYSYNC:
		return -EOPNOTSUPP;
	}
	return 0;
}

/* The 32-bit timestamp counter overflows every ~34.3 seconds; this task
 * forces periodic reads so that we don't miss any wraparounds.
 */
#define MV88E6XXX_TAI_OVERFLOW_PERIOD (HZ/4)
static void mv88e6xxx_ptp_overflow_check(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mv88e6xxx_chip *chip =
		container_of(dw, struct mv88e6xxx_chip, overflow_work);

	mutex_lock(&chip->reg_lock);
	timecounter_read(&chip->tstamp_tc);
	chip->last_overflow_check = jiffies;
	mutex_unlock(&chip->reg_lock);

	mv88e6xxx_ptp_trigger_lock(chip);

	schedule_delayed_work(&chip->overflow_work,
			      MV88E6XXX_TAI_OVERFLOW_PERIOD);
}

int mv88e6xxx_ptp_setup(struct mv88e6xxx_chip *chip)
{
	int i;

	/* Set up the cycle counter */
	memset(&chip->tstamp_cc, 0, sizeof(chip->tstamp_cc));
	chip->tstamp_cc.read	= mv88e6xxx_ptp_clock_read;
	chip->tstamp_cc.mask	= CYCLECOUNTER_MASK(32);
	chip->tstamp_cc.mult	= CC_MULT;
	chip->tstamp_cc.shift	= FRAC_SHIFT;

	timecounter_init(&chip->tstamp_tc, &chip->tstamp_cc,
			 ktime_to_ns(ktime_get_real()));

	chip->last_overflow_check = jiffies;

	INIT_DELAYED_WORK(&chip->overflow_work, mv88e6xxx_ptp_overflow_check);
	INIT_DELAYED_WORK(&chip->tai_event_work, mv88e6xxx_tai_event_work);

	chip->ptp_clock_info.owner = THIS_MODULE;
	snprintf(chip->ptp_clock_info.name, sizeof(chip->ptp_clock_info.name),
		 dev_name(chip->dev));

	/* Whole part of scaled ppm can't exceed max 16-bit number */
	chip->ptp_clock_info.max_adj	= 65535999;

	chip->ptp_clock_info.n_ext_ts	= 1;
	chip->ptp_clock_info.n_per_out	= 1;
	chip->ptp_clock_info.n_pins	= mv88e6xxx_num_gpio(chip);
	chip->ptp_clock_info.pps	= 1;

	for (i = 0; i < chip->ptp_clock_info.n_pins; ++i) {
		struct ptp_pin_desc *ppd = &chip->pin_config[i];

		snprintf(ppd->name, sizeof(ppd->name), "mv88e6xxx_gpio%d", i);
		ppd->index = i;
		ppd->func = PTP_PF_NONE;
	}
	chip->ptp_clock_info.pin_config = chip->pin_config;

	chip->perout.last_edge_time = 0;
	chip->perout.nominal_period = 0;

	chip->ptp_clock_info.adjfine	= mv88e6xxx_ptp_adjfine;
	chip->ptp_clock_info.adjtime	= mv88e6xxx_ptp_adjtime;
	chip->ptp_clock_info.gettime64	= mv88e6xxx_ptp_gettime;
	chip->ptp_clock_info.settime64	= mv88e6xxx_ptp_settime;
	chip->ptp_clock_info.enable	= mv88e6xxx_ptp_enable;
	chip->ptp_clock_info.verify	= mv88e6xxx_ptp_verify;

	chip->ptp_clock = ptp_clock_register(&chip->ptp_clock_info, chip->dev);
	if (IS_ERR(chip->ptp_clock))
		return PTR_ERR(chip->ptp_clock);

	schedule_delayed_work(&chip->overflow_work,
			      MV88E6XXX_TAI_OVERFLOW_PERIOD);

	return 0;
}

void mv88e6xxx_ptp_free(struct mv88e6xxx_chip *chip)
{
	if (chip->ptp_clock) {
		cancel_delayed_work_sync(&chip->overflow_work);
		cancel_delayed_work_sync(&chip->tai_event_work);

		ptp_clock_unregister(chip->ptp_clock);
		chip->ptp_clock = NULL;
	}
}
