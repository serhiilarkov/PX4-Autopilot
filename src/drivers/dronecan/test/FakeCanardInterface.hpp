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

#include "../CanardInterface.hpp"

/**
 * Off-target media backend for the contract tests. Captures transmitted frames
 * and serves queued frames back, with a loopback helper so an encoded transfer
 * can be fed straight back into the RX path -- exercising the full
 * encode -> TX queue -> media -> RX -> reassembly -> dispatch -> decode stack
 * with no hardware.
 */
class FakeCanardInterface : public CanardInterface
{
public:
	static constexpr uint32_t MaxFrames = 64;

	int16_t transmit(const CanardCANFrame &frame, int timeout_ms = 0) override
	{
		(void)timeout_ms;

		if (_tx_count < MaxFrames) {
			_tx[_tx_count++] = frame;
			return 1;
		}

		return -1;
	}

	int16_t receive(DronecanRxFrame *rxf) override
	{
		if (rxf == nullptr) {
			return -1;
		}

		if (_rx_head < _rx_count) {
			*rxf = _rx[_rx_head++];
			return 1;
		}

		return 0;
	}

	/// Move all captured TX frames into the RX queue (stamped ts), as if the bus
	/// echoed them back, then clear the TX capture.
	void loopbackTxToRx(uint64_t ts)
	{
		for (uint32_t i = 0; i < _tx_count && _rx_count < MaxFrames; i++) {
			_rx[_rx_count].timestamp_usec = ts;
			_rx[_rx_count].frame = _tx[i];
			_rx_count++;
		}

		_tx_count = 0;
	}

	uint32_t txCount() const { return _tx_count; }
	const CanardCANFrame &txFrame(uint32_t i) const { return _tx[i]; }

	void clear()
	{
		_tx_count = 0;
		_rx_count = 0;
		_rx_head = 0;
	}

private:
	CanardCANFrame  _tx[MaxFrames] {};
	DronecanRxFrame _rx[MaxFrames] {};
	uint32_t _tx_count {0};
	uint32_t _rx_count {0};
	uint32_t _rx_head {0};
};
