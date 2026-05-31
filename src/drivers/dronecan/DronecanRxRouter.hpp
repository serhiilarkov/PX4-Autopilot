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

#include "RxSubscriberBase.hpp"

/// Number of dispatch buckets. A single named compile-time constant (DESIGN Q1):
/// NOT cyphal's three-file preprocessor-summed UAVCAN_SUB_COUNT that silently
/// shrinks on a missing toggle. Sized generously for the full bridge+service set;
/// a power of two keeps the [id % N] cheap.
static constexpr uint8_t DC_RX_BUCKETS = 32;

/**
 * One registered acceptance entry. Intrusive: the node is a member of the
 * subscriber (or its registration helper), so registration never mallocs.
 */
struct RxHandler {
	uint16_t           data_type_id;
	CanardTransferType transfer_type;
	uint64_t           signature;
	RxSubscriberBase  *subscriber;
	RxHandler         *next;
};

/**
 * Per-instance, allocation-free dispatch registry. Reimplements ArduPilot's
 * allocation-free [data_type_id % N] bucketed HandlerList BEHAVIOR (broadcast ->
 * all matching handlers, service -> first matching) but per-instance, sidestepping
 * AP's static-global DEFINE_HANDLER_LIST_HEADS ODR trap.
 *
 * accept() (the shouldAccept path) and dispatch() (the onReception path) walk the
 * SAME bucket -- one source of truth, so the signature accept() writes always
 * matches the handler that will decode, closing the "accepted but wrong signature
 * -> CRC fail" class.
 */
class DronecanRxRouter
{
public:
	DronecanRxRouter() = default;

	/// Register an acceptance entry. The handler node storage is owned by the caller.
	void add(RxHandler *handler);

	/// shouldAccept path: find the first handler matching (dtid, transfer_type),
	/// write its signature to *out_signature, and return true. Returns false (never
	/// accepts) on a miss. MUST be side-effect-free: libcanard calls it a second time
	/// on mid-transfer frames.
	bool accept(uint16_t data_type_id, CanardTransferType transfer_type, uint64_t *out_signature) const;

	/// onReception path: deliver the transfer. Broadcast -> every matching handler;
	/// service (request/response) -> the first matching handler only. iface_id is the
	/// CAN interface the transfer arrived on, forwarded to each handler.
	void dispatch(const CanardRxTransfer &transfer, uint8_t iface_id) const;

private:
	RxHandler *_buckets[DC_RX_BUCKETS] {};
};
