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
					  u32 cycles, s16 picos)
{
	int err;
	u16 comp_config;

	if (picos < 0) {
		comp_config = BIT(15) | (-picos);
	} else {
		comp_config = picos;
	}

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_LO,
				  cycles & 0xffff);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_HI,
				  cycles >> 16);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CLOCK_COMP,
				  comp_config);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CFG,
				  0);

	return err;
}

static int mv88e6xxx_config_periodic_trig(struct mv88e6xxx_chip *chip,
					  u32 cycles, s16 picos, u32 start)
{
	int err;
	u16 global_config, comp_config;

	if (picos < 0) {
		comp_config = BIT(15) | (-picos);
	} else {
		comp_config = picos;
	}

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_LO,
				  cycles & 0xffff);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_GEN_AMOUNT_HI,
				  cycles >> 16);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CLOCK_COMP,
				  comp_config);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_LO,
				  start & 0xffff);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_TIME_HI,
				  start >> 16);
	if (err)
		return err;

	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_TRIG_CFG,
				  0);
	if (err)
		return err;




	chip->trig_config = MV88E6XXX_TAI_CFG_BLOCK_UPDATE |
		MV88E6XXX_TAI_CFG_TRIG_ENABLE;
	global_config = (chip->evcap_config | chip->trig_config);
	err = mv88e6xxx_tai_write(chip, MV88E6XXX_TAI_CFG, global_config);

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
	if (scaled_ppm == 0)
		return 0;

	return -EOPNOTSUPP;
}

static int mv88e6xxx_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj, s_period_cyc;
	u32 diff, mult, period_cyc;
	s16 picos;
	int neg_adj = 0;
	int err = 0;
	struct mv88e6xxx_chip *chip =
		container_of(ptp, struct mv88e6xxx_chip, ptp_clock_info);

	if (ppb == 0)
		return 0;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	mult = CC_MULT;
	adj = mult;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	mult = neg_adj ? mult - diff : mult + diff;

	if (mult == chip->tstamp_cc.mult)
		return 0;

	if (chip->perout.nominal_period) {
		s_period_cyc = (chip->perout.nominal_period <<
				(chip->tstamp_cc.shift+10)) / mult;
		period_cyc = s_period_cyc >> 10;
		picos = ((s_period_cyc & 0x3ff) * 8000) >> 10;
		if (picos > 4000) {
			period_cyc += 1;
			picos -= 8000;
		}
	} else {
		period_cyc = 0;
	}

	mutex_lock(&chip->reg_lock);
	timecounter_read(&chip->tstamp_tc);
	chip->tstamp_cc.mult = mult;
	if (period_cyc)
		err = mv88e6xxx_update_periodic_trig(chip, period_cyc, picos);
	mutex_unlock(&chip->reg_lock);

	if (period_cyc)
		dev_info(chip->dev, "Adj duty cycle to period %u, picos %d\n", period_cyc, picos);

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

	if (chip->perout.nominal_period == 0)
		return;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_tai_read(chip, MV88E6XXX_TAI_LOCK_STATUS, &status, 1);
	if (err)
		goto out;
	now_ns = timecounter_read(&chip->tstamp_tc);
	chip->last_overflow_check = jiffies;
	mult = chip->tstamp_cc.mult;
	shift = chip->tstamp_cc.shift;
	cycle = chip->tstamp_tc.cycle_last;
	frac_ns = chip->tstamp_tc.frac;
	mutex_unlock(&chip->reg_lock);

	edge_ns = chip->perout.last_edge_time;
	periods = (now_ns - edge_ns) / chip->perout.nominal_period;
	periods += 2;
	edge_ns += chip->perout.nominal_period * periods;

	wait_cycles = (((edge_ns - now_ns) << shift) - frac_ns + (1 << (shift-1))) / mult;
	cycle = cycle + wait_cycles - PEROUT_DELAY_CYCLES;

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

	chip->perout.last_edge_time = edge_ns;
	shift = (status >> 4) & 1;
	if (shift) {
		corr = status & 0x7;
		if (status & 0x8)
			corr = -corr;
	} else {
		corr = 0;
	}
	dev_info(chip->dev, "Trig lock at %llu wait %llu cyc %u prev valid: %d prev corr amt %d\n",
		 edge_ns, edge_ns - now_ns, (u32)wait_cycles, shift, corr);
 out:
	mutex_unlock(&chip->reg_lock);
	return;
}

static int mv88e6xxx_ptp_enable_perout(struct mv88e6xxx_chip *chip,
				       struct ptp_clock_request *rq, int on)
{
	struct timespec ts;
	struct timecounter *tc = &chip->tstamp_tc;
	u64 period_ns, now_ns, start_ns;
	u64 wait_ns, wait_cyc, s_period_cyc, frac_ns;
	u32 start_cyc, period_cyc, mult, shift;
	s16 picos;
	int pin;
	int err;

	pin = ptp_find_pin(chip->ptp_clock, PTP_PF_PEROUT, rq->perout.index);

	//dev_info(chip->dev, "%s perout on pin %d\n", on ? "enable" : "disable", pin);

	if (pin < 0)
		return -EBUSY;

	if (on) {
		ts.tv_sec = rq->perout.period.sec;
		ts.tv_nsec = rq->perout.period.nsec;
		period_ns = timespec_to_ns(&ts);

		if ((period_ns / 8) > U32_MAX) {
			dev_info(chip->dev, "period_ns too large\n");
			return -ERANGE;
		}

		ts.tv_sec = rq->perout.start.sec;
		ts.tv_nsec = rq->perout.start.nsec;
		start_ns = timespec_to_ns(&ts);

		mutex_lock(&chip->reg_lock);

		now_ns = timecounter_read(tc);
		chip->last_overflow_check = jiffies;
		mult = chip->tstamp_cc.mult;
		shift = chip->tstamp_cc.shift;
		frac_ns = chip->tstamp_tc.frac;

		mutex_unlock(&chip->reg_lock);

		if (now_ns > start_ns) {
			dev_info(chip->dev, "can't start in the past\n");
			return -ERANGE;
		}

		wait_ns = start_ns - now_ns;
		wait_cyc = ((wait_ns << shift) - frac_ns + (1 << (shift-1))) / mult;
		start_cyc = (tc->cycle_last + wait_cyc) - PEROUT_DELAY_CYCLES;
		if (start_cyc > tc->cc->mask) {
			dev_info(chip->dev, "too far in the future\n");
			return -ERANGE;
		}
		s_period_cyc = (period_ns << (shift+10)) / mult;
		period_cyc = s_period_cyc >> 10;
		picos = ((s_period_cyc & 0x3ff) * 8000) >> 10;
		if (picos > 4000) {
			period_cyc += 1;
			picos -= 8000;
		}

		dev_info(chip->dev, "Time: %llu (%llu cyc), start in %llu ns (%llu cyc) at ns %llu cycle %u period %u.%d\n",
			 now_ns, tc->cycle_last, wait_ns, wait_cyc, start_ns, start_cyc, (u32)(period_cyc << 3), picos);

		mutex_lock(&chip->reg_lock);

		chip->perout.nominal_period = period_ns;
		chip->perout.last_edge_time = start_ns;

		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_TRIG,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_OUT);
		if (err)
			goto out;

		err = mv88e6xxx_config_periodic_trig(chip, period_cyc, picos, start_cyc);
		if (err)
			goto out;

	} else {
		mutex_lock(&chip->reg_lock);

		mv88e6xxx_disable_trig(chip);
		err = mv88e6xxx_g2_set_gpio_config(
			chip, pin, MV88E6XXX_G2_SCRATCH_GPIO_MODE_GPIO,
			MV88E6XXX_G2_SCRATCH_GPIO_DIR_IN);

		chip->perout.nominal_period = 0;
		chip->perout.last_edge_time = 0;
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
#define MV88E6XXX_TAI_OVERFLOW_PERIOD (4*HZ)
static void mv88e6xxx_ptp_overflow_check(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mv88e6xxx_chip *chip =
		container_of(dw, struct mv88e6xxx_chip, overflow_work);

	mutex_lock(&chip->reg_lock);
	timecounter_read(&chip->tstamp_tc);
	chip->last_overflow_check = jiffies;
	mutex_unlock(&chip->reg_lock);

	//mv88e6xxx_ptp_trigger_lock(chip);

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

	chip->ptp_clock_info.max_adj	= 500000;

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

	/*
	chip->ptp_clock_info.adjfine	= mv88e6xxx_ptp_adjfine;
	*/
	chip->ptp_clock_info.adjfreq    = mv88e6xxx_ptp_adjfreq;
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
