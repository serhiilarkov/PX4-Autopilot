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

#include "CanardNuttXCDev.hpp"

#include <fcntl.h>
#include <poll.h>
#include <string.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "stm32_can.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

int CanardNuttXCDev::init()
{
	struct can_dev_s *can = stm32_caninitialize(1);

	if (can == nullptr) {
		PX4_ERR("Failed to get CAN interface");
		return -1;
	}

	/* Register the CAN driver at "/dev/can0" */
	const int ret = can_register("/dev/can0", can);

	if (ret < 0) {
		PX4_ERR("can_register failed: %d", ret);
		return -1;
	}

	_fd = ::open("/dev/can0", O_RDWR | O_NONBLOCK);

	return (_fd < 0) ? -1 : 0;
}

int16_t CanardNuttXCDev::transmit(const CanardCANFrame &frame, int timeout_ms)
{
	if (_fd < 0) {
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events |= POLLOUT;

	const int poll_result = poll(&fds, 1, timeout_ms);

	if (poll_result < 0) {
		return -1;
	}

	if (poll_result == 0) {
		return 0;
	}

	if ((fds.revents & POLLOUT) == 0) {
		return -1;
	}

	struct can_msg_s transmit_msg {};

	// Strip the EFF/RTR/ERR flags libcanard packs into id; the char driver carries
	// the 29-bit id plus the extended-id flag separately.
	transmit_msg.cm_hdr.ch_id = frame.id & CANARD_CAN_EXT_ID_MASK;

	transmit_msg.cm_hdr.ch_dlc = frame.data_len;

	transmit_msg.cm_hdr.ch_extid = 1;

	memcpy(transmit_msg.cm_data, frame.data, frame.data_len);

	const size_t msg_len = CAN_MSGLEN(transmit_msg.cm_hdr.ch_dlc);

	const ssize_t nbytes = ::write(_fd, &transmit_msg, msg_len);

	if (nbytes < 0 || (size_t)nbytes != msg_len) {
		return -1;
	}

	return 1;
}

int16_t CanardNuttXCDev::receive(DronecanRxFrame *rxf)
{
	if ((_fd < 0) || (rxf == nullptr)) {
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLIN;

	::poll(&fds, 1, 0);

	if (fds.revents & POLLIN) {
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			return -1;
		}

		// The char driver gives no hardware timestamp; stamp at read time.
		rxf->timestamp_usec = hrt_absolute_time();
		// libcanard rejects any frame without the extended-frame flag (canard.c:412).
		rxf->frame.id = receive_msg.cm_hdr.ch_id | CANARD_CAN_FRAME_EFF;
		rxf->frame.data_len = receive_msg.cm_hdr.ch_dlc;
		// Copy into the inline data[] -- the v0 frame owns its payload (no repoint).
		memcpy(rxf->frame.data, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
		return nbytes;
	}

	return 0;
}
