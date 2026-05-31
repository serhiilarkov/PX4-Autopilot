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

#include "CanardSocketCAN.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

namespace
{
uint64_t getMonotonicTimestampUSec()
{
	struct timespec ts {};

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}
}

int CanardSocketCAN::init()
{
	const char *const can_iface_name = "can0";

	struct sockaddr_can addr;
	struct ifreq ifr;

	/* open socket */
	if ((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("socket");
		return -1;
	}

	strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;

	/* RX Timestamping */
	if (setsockopt(_fd, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return -1;
	}

	/* NuttX Feature: Enable TX deadline when sending CAN frames
	 * When a deadline occurs the driver will remove the CAN frame
	 */
	if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return -1;
	}

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return -1;
	}

	// Setup TX msg
	_send_iov.iov_base = &_send_frame;
	_send_iov.iov_len = sizeof(struct can_frame);

	memset(&_send_control, 0x00, sizeof(_send_control));

	_send_msg.msg_iov = &_send_iov;
	_send_msg.msg_iovlen = 1;
	_send_msg.msg_control = &_send_control;
	_send_msg.msg_controllen = sizeof(_send_control);

	_send_cmsg = CMSG_FIRSTHDR(&_send_msg);
	_send_cmsg->cmsg_level = SOL_CAN_RAW;
	_send_cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	_send_cmsg->cmsg_len = sizeof(struct timeval);
	_send_tv = (struct timeval *)CMSG_DATA(_send_cmsg);

	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;
	_recv_iov.iov_len = sizeof(struct can_frame);

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);

	return 0;
}

int16_t CanardSocketCAN::transmit(const CanardCANFrame &frame, int timeout_ms)
{
	(void)timeout_ms;

	// Strip the EFF/RTR/ERR flags libcanard packs into id, then re-flag as extended.
	_send_frame.can_id = (frame.id & CANARD_CAN_EXT_ID_MASK) | CAN_EFF_FLAG;
	_send_frame.can_dlc = frame.data_len;
	memcpy(&_send_frame.data, frame.data, frame.data_len);

	// Convert the hrt-based TX deadline into a monotonic systick deadline; compensate
	// for the precision loss when converting hrt to systick.
	const uint64_t deadline_systick = getMonotonicTimestampUSec()
					  + (frame.deadline_usec - hrt_absolute_time())
					  + CONFIG_USEC_PER_TICK;

	_send_tv->tv_usec = deadline_systick % 1000000ULL;
	_send_tv->tv_sec = (deadline_systick - _send_tv->tv_usec) / 1000000ULL;

	return sendmsg(_fd, &_send_msg, 0);
}

int16_t CanardSocketCAN::receive(DronecanRxFrame *rxf)
{
	if (rxf == nullptr) {
		return -1;
	}

	const int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result < 0) {
		return result;
	}

	// libcanard rejects any frame without the extended-frame flag (canard.c:412).
	rxf->frame.id = (_recv_frame.can_id & CAN_EFF_MASK) | CANARD_CAN_FRAME_EFF;

	uint8_t data_len = _recv_frame.can_dlc;

	if (data_len > CANARD_CAN_FRAME_MAX_DATA_LEN) {
		data_len = CANARD_CAN_FRAME_MAX_DATA_LEN;
	}

	rxf->frame.data_len = data_len;
	// Copy into the inline data[] -- the v0 frame owns its payload (no repoint).
	memcpy(rxf->frame.data, _recv_frame.data, data_len);

	/* Read SO_TIMESTAMP value */
	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&_recv_msg);

	if (cmsg != nullptr && cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
		const struct timeval *tv = (struct timeval *)CMSG_DATA(cmsg);
		rxf->timestamp_usec = tv->tv_sec * 1000000ULL + tv->tv_usec;

	} else {
		rxf->timestamp_usec = hrt_absolute_time();
	}

	return result;
}
