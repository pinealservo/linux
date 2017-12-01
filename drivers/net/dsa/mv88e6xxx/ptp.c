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

static int mv88e6xxx_tai_read(struct mv88e6xxx_chip *chip, int addr,
			      u16 *data, int len)
{
	if (!chip->info->ops->avb_ops->tai_read)
		return -EOPNOTSUPP;

	return chip->info->ops->avb_ops->tai_read(chip, addr, data, len);
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

static int mv88e6xxx_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	if (scaled_ppm == 0)
		return 0;

	return -EOPNOTSUPP;
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

static int mv88e6xxx_ptp_enable(struct ptp_clock_info *ptp,
				struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static int mv88e6xxx_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
				enum ptp_pin_function func, unsigned int chan)
{
	return -EOPNOTSUPP;
}

/* The 32-bit timestamp counter overflows every ~34.3 seconds; this task
 * forces periodic reads so that we don't miss any wraparounds.
 */
#define MV88E6XXX_TAI_OVERFLOW_PERIOD (34 * HZ / 2)
static void mv88e6xxx_ptp_overflow_check(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mv88e6xxx_chip *chip =
		container_of(dw, struct mv88e6xxx_chip, overflow_work);
	bool timeout = time_is_before_jiffies(chip->last_overflow_check +
					      MV88E6XXX_TAI_OVERFLOW_PERIOD);

	if (timeout) {
		mutex_lock(&chip->reg_lock);
		timecounter_read(&chip->tstamp_tc);
		chip->last_overflow_check = jiffies;
		mutex_unlock(&chip->reg_lock);
	}

	schedule_delayed_work(&chip->overflow_work,
			      MV88E6XXX_TAI_OVERFLOW_PERIOD);
}

int mv88e6xxx_ptp_setup(struct mv88e6xxx_chip *chip)
{
	/* Set up the cycle counter */
	memset(&chip->tstamp_cc, 0, sizeof(chip->tstamp_cc));
	chip->tstamp_cc.read	= mv88e6xxx_ptp_clock_read;
	chip->tstamp_cc.mask	= CYCLECOUNTER_MASK(32);
	/* Raw timestamps are in units of 8-ns clock periods. */
	chip->tstamp_cc.mult	= 8;
	chip->tstamp_cc.shift	= 0;

	timecounter_init(&chip->tstamp_tc, &chip->tstamp_cc,
			 ktime_to_ns(ktime_get_real()));

	chip->last_overflow_check = jiffies;

	INIT_DELAYED_WORK(&chip->overflow_work, mv88e6xxx_ptp_overflow_check);

	chip->ptp_clock_info.owner = THIS_MODULE;
	snprintf(chip->ptp_clock_info.name, sizeof(chip->ptp_clock_info.name),
		 dev_name(chip->dev));
	chip->ptp_clock_info.max_adj	= 0;

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

		ptp_clock_unregister(chip->ptp_clock);
		chip->ptp_clock = NULL;
	}
}
