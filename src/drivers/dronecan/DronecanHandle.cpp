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

#include "DronecanHandle.hpp"

DronecanHandle::DronecanHandle(CanardInterface &iface, DronecanRxRouter &router) :
	_iface(iface),
	_router(router)
{
}

void DronecanHandle::init(uint32_t pool_bytes)
{
	if (pool_bytes > sizeof(_pool_storage)) {
		pool_bytes = sizeof(_pool_storage);
	}

	canardInit(&_canard, _pool_storage, pool_bytes,
		   &DronecanHandle::trampolineOnReception,
		   &DronecanHandle::trampolineShouldAccept,
		   this);
}

void DronecanHandle::receive()
{
	DronecanRxFrame rxf;

	// Push model: hand each frame to libcanard, which fires onReception inline.
	while (_iface.receive(&rxf) > 0) {
		canardHandleRxFrame(&_canard, &rxf.frame, rxf.timestamp_usec);
	}
}

void DronecanHandle::transmit()
{
	for (const CanardCANFrame *txf = canardPeekTxQueue(&_canard);
	     txf != nullptr;
	     txf = canardPeekTxQueue(&_canard)) {

		const int16_t res = _iface.transmit(*txf, 0);

		if (res <= 0) {
			// Media not ready or error: leave the frame queued for the next pump.
			break;
		}

		canardPopTxQueue(&_canard);
	}
}

int16_t DronecanHandle::broadcast(const TypeDescriptor &desc, const uint8_t *payload, uint16_t payload_len,
				  uint8_t *inout_transfer_id, uint8_t priority, uint64_t deadline_usec)
{
	// The stack-local CanardTxTransfer, built per call -- never a reused member, so
	// there is no shared-state hazard between concurrent publishers.
	CanardTxTransfer transfer;
	canardInitTxTransfer(&transfer);
	transfer.transfer_type = CanardTransferTypeBroadcast;
	transfer.data_type_signature = desc.signature;
	transfer.data_type_id = desc.data_type_id;
	transfer.inout_transfer_id = inout_transfer_id;
	transfer.priority = priority;
	transfer.payload = payload;
	transfer.payload_len = payload_len;
	transfer.tao = true;
	transfer.deadline_usec = deadline_usec;

	const int16_t res = canardBroadcastObj(&_canard, &transfer);

	if (res <= 0) {
		// OOM (shared pool exhausted) or full bus -- never silent (DC_POOL_EXHAUSTED).
		_pool_exhausted_count++;
	}

	return res;
}

int16_t DronecanHandle::requestOrRespond(uint8_t destination_node_id, const TypeDescriptor &desc,
		CanardTransferType transfer_type, const uint8_t *payload, uint16_t payload_len,
		uint8_t *inout_transfer_id, uint8_t priority, uint64_t deadline_usec)
{
	CanardTxTransfer transfer;
	canardInitTxTransfer(&transfer);
	transfer.transfer_type = transfer_type;
	transfer.data_type_signature = desc.signature;
	transfer.data_type_id = desc.data_type_id;
	transfer.inout_transfer_id = inout_transfer_id;
	transfer.priority = priority;
	transfer.payload = payload;
	transfer.payload_len = payload_len;
	transfer.tao = true;
	transfer.deadline_usec = deadline_usec;

	const int16_t res = canardRequestOrRespondObj(&_canard, destination_node_id, &transfer);

	if (res <= 0) {
		_pool_exhausted_count++;
	}

	return res;
}

bool DronecanHandle::trampolineShouldAccept(const CanardInstance *ins, uint64_t *out_signature,
		uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id)
{
	(void)source_node_id;

	DronecanHandle *self = static_cast<DronecanHandle *>(canardGetUserReference(ins));

	if (self == nullptr) {
		return false;
	}

	// Side-effect-free: libcanard calls this a second time on mid-transfer frames.
	return self->_router.accept(data_type_id, transfer_type, out_signature);
}

void DronecanHandle::trampolineOnReception(CanardInstance *ins, CanardRxTransfer *transfer)
{
	DronecanHandle *self = static_cast<DronecanHandle *>(canardGetUserReference(ins));

	if (self == nullptr || transfer == nullptr) {
		return;
	}

	self->_router.dispatch(*transfer);
}
