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

#include <stdint.h>

class DronecanHandle;

/**
 * The trait SEAM kept as data, not templates (DESIGN Q1). One ~24-byte POD per
 * message type, populated from the generated <TYPE>_ID/_SIGNATURE/_MAX_SIZE/_NAME
 * #defines. This is rodata, not code -- the per-type flash cost stays flat.
 */
struct TypeDescriptor {
	uint16_t data_type_id;
	uint64_t signature;
	uint16_t max_size;
	CanardTransferType transfer_type;
	const char *name;
};

/// Generated decode signature, type-erased. The real function is
/// bool <type>_decode(const CanardRxTransfer *, struct <type> *) and returns
/// TRUE ON FAILURE (legacy libcanard contract).
typedef bool (*DroneCANDecodeFn)(const CanardRxTransfer *transfer, void *out_msg);

/// Generated encode signature, type-erased. The real function is
/// uint32_t <type>_encode(struct <type> *, uint8_t *buffer, bool tao) and returns
/// the encoded byte length, 0 ON FAILURE.
typedef uint32_t (*DroneCANEncodeFn)(void *msg, uint8_t *buffer, bool tao);

/**
 * The paper-thin codec shim. These free functions are the ONLY places in the
 * whole binary that interpret libcanard's two easy-to-forget return-code
 * contracts. Bridge authors call them with normal polarity and physically
 * cannot open-code the negation.
 */
namespace dronecan_codec
{

/**
 * The ONE place libcanard's TRUE-on-failure decode result is inverted.
 * Returns true on SUCCESS (normal polarity for every caller).
 */
bool decodeTransfer(const CanardRxTransfer &transfer, void *out_msg, DroneCANDecodeFn fn);

/**
 * The ONE place the generated encode is called and its 0-length failure is
 * checked. Encodes msg into buffer (which must be >= max_size bytes) with TAO
 * enabled. Returns the encoded length (>0) or -1 on failure.
 */
int32_t encodePayload(const void *msg, uint8_t *buffer, DroneCANEncodeFn fn);

/**
 * Encode + enqueue a broadcast transfer. Funnels through encodePayload (the one
 * encode call site) then DronecanHandle::broadcast (the one place the canard
 * CanardTxTransfer is built). buffer must be >= desc.max_size bytes and is owned
 * by the caller (typically a per-type publisher member). inout_transfer_id must
 * point at a persistent (never stack, never shared) per-publisher transfer id.
 * Returns the number of frames enqueued, or <0 on encode failure / pool exhaustion.
 */
int32_t encodeBroadcast(DronecanHandle &handle, const TypeDescriptor &desc, const void *msg,
			uint8_t *buffer, uint8_t *inout_transfer_id, uint8_t priority,
			DroneCANEncodeFn fn, uint64_t deadline_usec);

} // namespace dronecan_codec
