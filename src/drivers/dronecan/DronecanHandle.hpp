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

#include "CanardInterface.hpp"
#include "DroneCANCodec.hpp"
#include "DronecanRxRouter.hpp"

/**
 * The ONLY PX4<->libcanard boundary. Owns the CanardInstance, the single static
 * memory pool, and the two static trampolines; delegates dispatch to an injected
 * DronecanRxRouter and media I/O to an injected CanardInterface.
 *
 * Deliberately platform-free (canard + std only, no perf/hrt/log): the whole
 * stack is drivable off-target from a fake media interface. Timestamps are passed
 * in by the caller (the node owns hrt); pool exhaustion is exposed as an
 * observable counter that the node surfaces via perf/log.
 */
class DronecanHandle
{
public:
	/// DC_POOL_MAX: the static pool ceiling. The live pool handed to canardInit is the
	/// runtime DC_POOL bound (default 8192, ~204 blocks at 40 B), clamped to this; sizing
	/// the static buffer larger lets a high-node-count bus grow DC_POOL without a recompile
	/// (DESIGN Q5/§3). The node passes DC_POOL into init(); the default is the full ceiling.
	static constexpr uint32_t PoolStorageBytes = 16384;

	DronecanHandle(CanardInterface &iface, DronecanRxRouter &router);

	/// canardInit the instance over the static pool. pool_bytes is clamped to the
	/// static storage size. Anonymous (node id 0) until setNodeID().
	void init(uint32_t pool_bytes = PoolStorageBytes);

	void setNodeID(uint8_t node_id) { canardSetLocalNodeID(&_canard, node_id); }
	uint8_t nodeID() const { return canardGetLocalNodeID(&_canard); }

	/// Drain all available media frames into libcanard. onReception callbacks fire
	/// inline (push model), enqueuing any service responses for the next transmit().
	void receive();

	/// Drain the libcanard TX queue to the media.
	void transmit();

	/// Periodic stale-transfer cleanup; call about once per second.
	void cleanupStale(uint64_t now_usec) { canardCleanupStaleTransfers(&_canard, now_usec); }

	/// Build the stack-local CanardTxTransfer and enqueue a broadcast. The ONE place
	/// a CanardTxTransfer is constructed. Returns frames enqueued, or <0 on OOM.
	int16_t broadcast(const TypeDescriptor &desc, const uint8_t *payload, uint16_t payload_len,
			  uint8_t *inout_transfer_id, uint8_t priority, uint64_t deadline_usec);

	/// As above, for a service request or response. Requires a set node id.
	int16_t requestOrRespond(uint8_t destination_node_id, const TypeDescriptor &desc,
				 CanardTransferType transfer_type, const uint8_t *payload, uint16_t payload_len,
				 uint8_t *inout_transfer_id, uint8_t priority, uint64_t deadline_usec);

	CanardPoolAllocatorStatistics poolStats() { return canardGetPoolAllocatorStatistics(&_canard); }

	/// Count of TX enqueue attempts that failed (OOM / DC_POOL_EXHAUSTED). Never
	/// silent: the node surfaces this via perf counter + rate-limited log.
	uint32_t poolExhaustedCount() const { return _pool_exhausted_count; }

	CanardInstance *instance() { return &_canard; }

private:
	/// user_reference -> DronecanHandle* trampolines (AP_Canard_iface pattern).
	static bool trampolineShouldAccept(const CanardInstance *ins, uint64_t *out_signature,
					   uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id);
	static void trampolineOnReception(CanardInstance *ins, CanardRxTransfer *transfer);

	CanardInterface  &_iface;
	DronecanRxRouter &_router;

	CanardInstance _canard {};

	// Word-aligned static pool storage, a per-instance member (matches AP's uint32_t[]
	// arena, not a static-global). The single arena canardInit slices into 40 B blocks
	// shared by RX reassembly and TX enqueue.
	uint32_t _pool_storage[PoolStorageBytes / sizeof(uint32_t)] {};

	uint32_t _pool_exhausted_count {0};
};
