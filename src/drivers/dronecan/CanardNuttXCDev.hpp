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

/**
 * NuttX upper-half CAN char-device media backend (CONFIG_CAN_EXTID, e.g. the
 * bxCAN /dev/can0 path). Ported from the cyphal CanardNuttXCDev, re-typed to the
 * legacy v0 CanardCANFrame: the frame carries an inline data[8] array (not a
 * repointed pointer), and the 29-bit id is masked on TX / OR'd with the EFF flag
 * on RX so libcanard accepts it.
 */
class CanardNuttXCDev : public CanardInterface
{
public:
	CanardNuttXCDev() = default;
	~CanardNuttXCDev() override = default;

	/// Bring up the STM32 CAN device and open /dev/can0 non-blocking.
	/// Returns 0 on success, -1 on error.
	int init() override;

	/// Transmit one frame. Returns 1 when the frame was written, 0 when the media
	/// is not ready (try again later), <0 on error.
	int16_t transmit(const CanardCANFrame &frame, int timeout_ms = 0) override;

	/// Receive one frame into rxf. Returns >0 when a frame was read, 0 when none
	/// is available, <0 on error.
	int16_t receive(DronecanRxFrame *rxf) override;

private:
	int _fd{-1};
};
