/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Neutralized for PX4: this STM32H7 FDCAN register driver was extracted from
 * libuavcan and is now framework-neutral. It is written against the px4can shim
 * (no libuavcan protocol dependency); the alias below resolves the driver's
 * historical `uavcan::` type spellings to px4can so the register code is unchanged.
 */

#pragma once

#include <px4_platform_common/px4can.hpp>

// Resolve the driver's `uavcan::` driver-layer type names to the neutral shim. The
// dronecan/uavcan stacks are mutually exclusive at link time, and this driver never
// includes the real libuavcan, so this alias never collides with a real `uavcan`.
namespace uavcan = px4can;

// libuavcan-compat spellings still used by the register code.
#ifndef UAVCAN_NULLPTR
# define UAVCAN_NULLPTR nullptr
#endif
#ifndef UAVCAN_ASSERT
# define UAVCAN_ASSERT(x) ((void)0)
#endif

/**
 * This driver targets NuttX only.
 */
#ifndef UAVCAN_STM32H7_NUTTX
# define UAVCAN_STM32H7_NUTTX 1
#endif

/**
 * Number of interfaces (1 or 2). Override from the consumer's build if needed.
 */
#ifndef UAVCAN_STM32H7_NUM_IFACES
# define UAVCAN_STM32H7_NUM_IFACES 1
#endif

#if (UAVCAN_STM32H7_NUM_IFACES != 1 && UAVCAN_STM32H7_NUM_IFACES != 2)
# error "UAVCAN_STM32H7_NUM_IFACES must be set to either 1 or 2"
#endif
