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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <canard.h>

#include "CanardInterface.hpp"

/**
 * SocketCAN media backend (CONFIG_NET_CAN, e.g. the STM32H7 FDCAN socket driver).
 * Ported from the cyphal CanardSocketCAN, re-typed to the legacy v0 CanardCANFrame
 * and pinned to classic 8-byte CAN (CANARD_ENABLE_CANFD=0). The received payload is
 * memcpy'd into the frame's inline data[] -- the cyphal version repointed the modern
 * CanardFrame::payload at the recv buffer, which dangles once recvmsg() reuses it.
 */
class CanardSocketCAN : public CanardInterface
{
public:
	CanardSocketCAN() = default;
	~CanardSocketCAN() override = default;

	/// Open and bind a SocketCAN raw socket on can0. Returns 0 on success, -1 on error.
	int init() override;

	int close() override
	{
		return ::close(_fd);
	}

	/// Transmit one frame with a TX deadline. Returns the sendmsg() result.
	int16_t transmit(const CanardCANFrame &frame, int timeout_ms = 0) override;

	/// Receive one frame into rxf. Returns >0 when a frame was read, <0 when none is
	/// available (EAGAIN) or on error.
	int16_t receive(DronecanRxFrame *rxf) override;

private:
	int _fd{-1};

	//// Send msg structure
	struct iovec       _send_iov {};
	struct can_frame   _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr    *_send_cmsg {};
	struct timeval    *_send_tv {};  /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct can_frame   _recv_frame {};
	struct msghdr      _recv_msg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};
};
