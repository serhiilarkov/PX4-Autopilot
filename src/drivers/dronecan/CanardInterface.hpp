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

#include <canard.h>

/**
 * One received CAN frame plus the timestamp libcanard needs to build RX state.
 *
 * The legacy v0 frame type is CanardCANFrame (29-bit id + up to 8 data bytes),
 * not the modern Cyphal CanardFrame. This is the v0 re-typing of cyphal's
 * CanardRxFrame.
 */
struct DronecanRxFrame {
	uint64_t timestamp_usec;
	CanardCANFrame frame;
};

/**
 * Media abstraction for the legacy DroneCAN-v0 transport. Concrete backends:
 *  - CanardNuttXCDev   (CONFIG_CAN, on-target)      [P4.3]
 *  - CanardSocketCAN   (CONFIG_NET_CAN, SITL)       [P4.3]
 *  - FakeCanardInterface (test/, off-target loopback)
 *
 * DronecanHandle is constructor-injected with one of these so the whole stack
 * is drivable off-target.
 */
class CanardInterface
{
public:
	CanardInterface() = default;
	virtual ~CanardInterface() = default;

	virtual int init() { return 0; }

	virtual int close() { return 0; }

	/// Transmit one frame. Returns >0 when the frame was accepted by the media,
	/// 0 when the media is not ready (try again later), <0 on error.
	virtual int16_t transmit(const CanardCANFrame &frame, int timeout_ms = 0) = 0;

	/// Receive one frame into rxf. Returns >0 when a frame was read, 0 when none
	/// is available, <0 on error.
	virtual int16_t receive(DronecanRxFrame *rxf) = 0;
};
