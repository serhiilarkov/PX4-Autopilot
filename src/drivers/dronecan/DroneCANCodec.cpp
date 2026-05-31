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

#include "DroneCANCodec.hpp"
#include "DronecanHandle.hpp"

namespace dronecan_codec
{

bool decodeTransfer(const CanardRxTransfer &transfer, void *out_msg, DroneCANDecodeFn fn)
{
	// libcanard's generated decode returns TRUE ON FAILURE. This negation is the
	// single point of truth: every caller writes if (decode(...)) { use(struct) }
	// with normal polarity and cannot forget the inversion.
	const bool failed = fn(&transfer, out_msg);
	return !failed;
}

int32_t encodePayload(const void *msg, uint8_t *buffer, DroneCANEncodeFn fn)
{
	// Generated encode returns the encoded byte length, 0 ON FAILURE. TAO is always
	// enabled (CANARD_ENABLE_TAO_OPTION=1). This is the single encode call site and
	// the single place the length-return failure is interpreted.
	const uint32_t len = fn(const_cast<void *>(msg), buffer, true);

	if (len == 0) {
		return -1;
	}

	return static_cast<int32_t>(len);
}

int32_t encodeBroadcast(DronecanHandle &handle, const TypeDescriptor &desc, const void *msg,
			uint8_t *buffer, uint8_t *inout_transfer_id, uint8_t priority,
			DroneCANEncodeFn fn, uint64_t deadline_usec)
{
	const int32_t len = encodePayload(msg, buffer, fn);

	if (len < 0) {
		return -1;
	}

	return handle.broadcast(desc, buffer, static_cast<uint16_t>(len), inout_transfer_id, priority, deadline_usec);
}

} // namespace dronecan_codec
