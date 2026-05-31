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

#include "DronecanRxRouter.hpp"

void DronecanRxRouter::add(RxHandler *handler)
{
	if (handler == nullptr) {
		return;
	}

	const uint8_t bucket = handler->data_type_id % DC_RX_BUCKETS;
	handler->next = _buckets[bucket];
	_buckets[bucket] = handler;
}

bool DronecanRxRouter::accept(uint16_t data_type_id, CanardTransferType transfer_type, uint64_t *out_signature) const
{
	const uint8_t bucket = data_type_id % DC_RX_BUCKETS;

	for (const RxHandler *h = _buckets[bucket]; h != nullptr; h = h->next) {
		if (h->data_type_id == data_type_id && h->transfer_type == transfer_type) {
			if (out_signature != nullptr) {
				*out_signature = h->signature;
			}

			return true;
		}
	}

	return false;
}

void DronecanRxRouter::dispatch(const CanardRxTransfer &transfer) const
{
	const uint8_t bucket = transfer.data_type_id % DC_RX_BUCKETS;
	const bool is_service = (transfer.transfer_type != CanardTransferTypeBroadcast);

	for (const RxHandler *h = _buckets[bucket]; h != nullptr; h = h->next) {
		if (h->data_type_id == transfer.data_type_id
		    && static_cast<uint8_t>(h->transfer_type) == transfer.transfer_type) {

			h->subscriber->handle(transfer);

			// Broadcast delivers to every matching handler; a service transfer is
			// consumed by the first matching handler only.
			if (is_service) {
				break;
			}
		}
	}
}
