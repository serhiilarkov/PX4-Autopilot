/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Neutralized for PX4: the original libuavcan clock drove a dedicated general-purpose
 * timer. This shared driver instead reads the always-running PX4 high-resolution timer
 * (hrt), so it needs no extra timer resource and shares PX4's monotonic timebase.
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>

namespace uavcan_stm32h7
{
namespace clock
{
/// No-op: the monotonic source is the always-running PX4 hrt.
void init();

/// Monotonic time (microsecond resolution) from hrt.
uavcan::MonotonicTime getMonotonic();

/// UTC time. Equal to monotonic here -- the shared driver does not run UTC sync.
uavcan::UtcTime getUtc();

/// Monotonic microseconds, callable from CAN ISR context (hrt is ISR-safe).
uavcan::uint64_t getUtcUSecFromCanInterrupt();

/// UTC adjustment hook, unused by the shared driver.
void adjustUtc(uavcan::UtcDuration adjustment);
}
}
