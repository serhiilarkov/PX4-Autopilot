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

#include "CanardFdcanIface.hpp"

#include <drivers/drv_hrt.h>
#include <string.h>

int CanardFdcanIface::init(uint32_t bitrate)
{
	uavcan_stm32h7::clock::init();

	// CanInitHelper has both init(uint32_t) (fixed bitrate) and init(uint32_t&) (auto-
	// detect). Pass a prvalue so only the fixed-bitrate overload is viable.
	const int res = _can.init(uint32_t(bitrate));

	if (res < 0) {
		return -1;
	}

	_iface = _can.driver.getIface(0);

	return (_iface != nullptr) ? 0 : -1;
}

void CanardFdcanIface::registerBusEventCallback(void (*handler)())
{
	_can.driver.updateEvent().registerSignalCallback(handler);
}

int16_t CanardFdcanIface::transmit(const CanardCANFrame &frame, int timeout_ms)
{
	(void)timeout_ms;

	if (_iface == nullptr) {
		return -1;
	}

	px4can::CanFrame out_frame;
	out_frame.id = frame.id; // FlagEFF + 29-bit id are bit-identical to libcanard

	uint8_t dlc = frame.data_len;

	if (dlc > px4can::CanFrame::MaxDataLen) {
		dlc = px4can::CanFrame::MaxDataLen;
	}

	out_frame.dlc = dlc;
	memcpy(out_frame.data, frame.data, dlc);

	// frame.deadline_usec is an hrt-based absolute deadline; the driver clock is also
	// hrt, so the remaining-time conversion is exact.
	int64_t remaining_usec = (int64_t)frame.deadline_usec - (int64_t)hrt_absolute_time();

	if (remaining_usec < 0) {
		remaining_usec = 0;
	}

	const px4can::MonotonicTime deadline =
		uavcan_stm32h7::clock::getMonotonic() + px4can::MonotonicDuration::fromUSec(remaining_usec);

	// 1 = enqueued, 0 = no TX mailbox free (retry next pump), <0 = error.
	return _iface->send(out_frame, deadline, 0);
}

int16_t CanardFdcanIface::receive(DronecanRxFrame *rxf)
{
	if (_iface == nullptr || rxf == nullptr) {
		return -1;
	}

	px4can::CanFrame in_frame;
	px4can::MonotonicTime ts_monotonic;
	px4can::UtcTime ts_utc;
	px4can::CanIOFlags flags = 0;

	const int16_t res = _iface->receive(in_frame, ts_monotonic, ts_utc, flags);

	if (res <= 0) {
		// 0 == RX queue empty, <0 == error.
		return res;
	}

	rxf->frame.id = in_frame.id | CANARD_CAN_FRAME_EFF;

	uint8_t dlc = in_frame.dlc;

	if (dlc > CANARD_CAN_FRAME_MAX_DATA_LEN) {
		dlc = CANARD_CAN_FRAME_MAX_DATA_LEN;
	}

	rxf->frame.data_len = dlc;
	memcpy(rxf->frame.data, in_frame.data, dlc);

	// Stamp in the canard timebase (hrt) -- the node runs cleanupStale() on hrt.
	rxf->timestamp_usec = hrt_absolute_time();

	return res;
}
