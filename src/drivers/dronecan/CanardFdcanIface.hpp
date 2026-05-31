/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>

#include <canard.h>

#include "CanardInterface.hpp"

#include <uavcan_stm32h7/can.hpp>
#include <uavcan_stm32h7/clock.hpp>

/**
 * DroneCAN media backend over the shared, framework-neutral STM32H7 FDCAN driver
 * (platforms/nuttx/src/px4/stm/stm32h7/can, written against the px4can shim -- no
 * libuavcan dependency). Converts between the v0 CanardCANFrame and px4can::CanFrame
 * (identical layout) and routes the driver's CAN-RX-IRQ BusEvent to the node's
 * ScheduleNow() trampoline.
 */
class CanardFdcanIface : public CanardInterface
{
public:
	CanardFdcanIface() = default;
	~CanardFdcanIface() override = default;

	using CanardInterface::init; // keep the no-arg base overload visible

	/// Bring up the FDCAN hardware at the given bitrate. Returns 0 on success, -1 on error.
	int init(uint32_t bitrate);

	/// Route the driver's CAN-RX-IRQ BusEvent to a node trampoline (ScheduleNow()).
	void registerBusEventCallback(void (*handler)());

	int16_t transmit(const CanardCANFrame &frame, int timeout_ms = 0) override;
	int16_t receive(DronecanRxFrame *rxf) override;

private:
	// RX slots per iface. 64 * ~145 us/frame ~= 9 ms headroom against the 3 ms tick.
	static constexpr unsigned RxQueueLenPerIface = 64;

	uavcan_stm32h7::CanInitHelper<RxQueueLenPerIface> _can;
	px4can::ICanIface *_iface{nullptr};
};
