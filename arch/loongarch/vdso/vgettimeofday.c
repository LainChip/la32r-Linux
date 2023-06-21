// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * LoongArch userspace implementations of gettimeofday() and similar.
 *
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */
#include <linux/time.h>
#include <linux/types.h>

int __vdso_clock_gettime(clockid_t clock,
			 struct __kernel_timespec *ts)
{
	return __cvdso_clock_gettime(clock, ts);
}

int __vdso_gettimeofday(struct __kernel_old_timeval *tv,
			struct timezone *tz)
{
	return __cvdso_gettimeofday(tv, tz);
}

int __vdso_clock_getres(clockid_t clock_id,
			struct __kernel_timespec *res)
{
	return __cvdso_clock_getres(clock_id, res);
}
