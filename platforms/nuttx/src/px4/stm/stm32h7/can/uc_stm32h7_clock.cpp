/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Neutralized for PX4: hrt-backed monotonic clock for the shared STM32H7 FDCAN
 * driver. Replaces the original dedicated-timer (TIMx) implementation -- no extra
 * hardware timer is consumed, and the driver shares PX4's monotonic timebase.
 */

#include <uavcan_stm32h7/clock.hpp>

#include <drivers/drv_hrt.h>

namespace uavcan_stm32h7
{
namespace clock
{

void init() { }

uavcan::MonotonicTime getMonotonic()
{
	return uavcan::MonotonicTime::fromUSec(hrt_absolute_time());
}

uavcan::UtcTime getUtc()
{
	return uavcan::UtcTime::fromUSec(hrt_absolute_time());
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return hrt_absolute_time();
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	(void)adjustment;
}

}
}
