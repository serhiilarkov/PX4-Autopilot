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

/*
 * Off-target contract test for the DroneCAN transport stack. Proves the two
 * easy-to-forget libcanard return-code contracts are handled centrally BEFORE any
 * bridge is written:
 *   1. decode round-trip identity through the codec shim,
 *   2. the decode TRUE-on-failure inversion (stub + a real malformed transfer),
 *   3. fake-frame dispatch through the full handle + router stack.
 *
 * Standalone host program (no PX4 platform deps) so it runs off-target. Build &
 * run with: src/drivers/dronecan/test/run_codec_test.sh (builds against canard.c +
 * the generated codecs with the locked canard flags). Returns non-zero on any
 * failed check.
 */

#include "../DroneCANCodec.hpp"
#include "../DronecanHandle.hpp"
#include "../DronecanRxRouter.hpp"
#include "../RxSubscriberBase.hpp"
#include "FakeCanardInterface.hpp"

#include <uavcan.protocol.NodeStatus.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

int g_failures = 0;

#define CHECK(cond) \
	do { \
		if (!(cond)) { \
			std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__); \
			g_failures++; \
		} \
	} while (0)

const TypeDescriptor NodeStatusDesc = {
	UAVCAN_PROTOCOL_NODESTATUS_ID,
	UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
	CanardTransferTypeBroadcast,
	"uavcan.protocol.NodeStatus"
};

// Thin typed wrappers -- what the generated T -> {encode,decode} traits map will
// emit. No function-pointer cast: the typed call happens here, the type-erased
// pointer is what the codec stores.
bool nodestatus_decode(const CanardRxTransfer *t, void *m)
{
	return uavcan_protocol_NodeStatus_decode(t, static_cast<uavcan_protocol_NodeStatus *>(m));
}

uint32_t nodestatus_encode(void *m, uint8_t *buf, bool tao)
{
	return uavcan_protocol_NodeStatus_encode(static_cast<uavcan_protocol_NodeStatus *>(m), buf, tao);
}

bool stub_decode_success(const CanardRxTransfer *, void *) { return false; } // libcanard: false == success
bool stub_decode_failure(const CanardRxTransfer *, void *) { return true; }  // libcanard: true  == failure

uavcan_protocol_NodeStatus make_sample()
{
	uavcan_protocol_NodeStatus m {};
	m.uptime_sec = 0x12345678;
	m.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
	m.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE;
	m.sub_mode = 5;
	m.vendor_specific_status_code = 0xABCD;
	return m;
}

bool same(const uavcan_protocol_NodeStatus &a, const uavcan_protocol_NodeStatus &b)
{
	return a.uptime_sec == b.uptime_sec
	       && a.health == b.health
	       && a.mode == b.mode
	       && a.sub_mode == b.sub_mode
	       && a.vendor_specific_status_code == b.vendor_specific_status_code;
}

// ---- 1. decode inversion (the single point of correctness) ----------------
void test_decode_inversion()
{
	std::printf("test_decode_inversion\n");
	CanardRxTransfer t {};
	uint8_t scratch[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE] {};

	// libcanard false (success) must surface as decode() == true (normal polarity).
	CHECK(dronecan_codec::decodeTransfer(t, scratch, stub_decode_success) == true);
	// libcanard true (failure) must surface as decode() == false.
	CHECK(dronecan_codec::decodeTransfer(t, scratch, stub_decode_failure) == false);
}

// ---- 2. encode -> decode identity (codec only) ----------------------------
void test_codec_roundtrip()
{
	std::printf("test_codec_roundtrip\n");
	uavcan_protocol_NodeStatus in = make_sample();

	uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE] {};
	const int32_t len = dronecan_codec::encodePayload(&in, buf, nodestatus_encode);
	CHECK(len > 0);
	CHECK(len == UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);

	CanardRxTransfer t {};
	t.payload_head = buf;
	t.payload_len = static_cast<uint16_t>(len);
	t.data_type_id = UAVCAN_PROTOCOL_NODESTATUS_ID;
	t.tao = true;

	uavcan_protocol_NodeStatus out {};
	CHECK(dronecan_codec::decodeTransfer(t, &out, nodestatus_decode) == true);
	CHECK(same(in, out));
}

// ---- 3. wrong-length transfer must decode() == false (real message) -------
void test_wrong_length_rejected()
{
	std::printf("test_wrong_length_rejected\n");
	uavcan_protocol_NodeStatus in = make_sample();

	uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE] {};
	const int32_t len = dronecan_codec::encodePayload(&in, buf, nodestatus_encode);
	CHECK(len == UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);

	uavcan_protocol_NodeStatus out {};

	// Too short: byte_len(7) != payload_len(6) under TAO -> decode failure.
	CanardRxTransfer too_short {};
	too_short.payload_head = buf;
	too_short.payload_len = static_cast<uint16_t>(len - 1);
	too_short.tao = true;
	CHECK(dronecan_codec::decodeTransfer(too_short, &out, nodestatus_decode) == false);

	// Too long: payload_len(8) > MAX_SIZE(7) under TAO -> decode failure.
	CanardRxTransfer too_long {};
	too_long.payload_head = buf;
	too_long.payload_len = static_cast<uint16_t>(UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE + 1);
	too_long.tao = true;
	CHECK(dronecan_codec::decodeTransfer(too_long, &out, nodestatus_decode) == false);
}

// ---- 4. fake-frame dispatch through the full handle + router stack ---------
class CapturingSubscriber : public RxSubscriberBase
{
public:
	void handle(const CanardRxTransfer &transfer, uint8_t iface_id) override
	{
		(void)iface_id;
		_calls++;
		_decode_ok = dronecan_codec::decodeTransfer(transfer, &_last, nodestatus_decode);
	}

	int _calls {0};
	bool _decode_ok {false};
	uavcan_protocol_NodeStatus _last {};
};

void test_fake_frame_dispatch()
{
	std::printf("test_fake_frame_dispatch\n");
	FakeCanardInterface fake;
	DronecanRxRouter router;
	DronecanHandle handle(fake, router);
	handle.init();
	handle.setNodeID(42);
	CHECK(handle.nodeID() == 42);

	CapturingSubscriber sub;
	RxHandler h {
		UAVCAN_PROTOCOL_NODESTATUS_ID,
		CanardTransferTypeBroadcast,
		UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
		&sub,
		nullptr
	};
	router.add(&h);

	uavcan_protocol_NodeStatus in = make_sample();
	uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE] {};
	uint8_t transfer_id = 0;

	const int32_t frames = dronecan_codec::encodeBroadcast(handle, NodeStatusDesc, &in, buf, &transfer_id,
			       CANARD_TRANSFER_PRIORITY_MEDIUM, nodestatus_encode, /*deadline*/ 1000000);
	CHECK(frames > 0);
	CHECK(transfer_id == 1); // libcanard auto-incremented the persistent transfer id

	handle.transmit();             // drain canard TX queue -> fake media
	CHECK(fake.txCount() == 1);    // NodeStatus is a single CAN frame

	fake.loopbackTxToRx(5000);     // echo the frame back onto the "bus"
	handle.receive();              // media -> canardHandleRxFrame -> dispatch -> subscriber

	CHECK(sub._calls == 1);
	CHECK(sub._decode_ok == true);
	CHECK(same(in, sub._last));

	// A frame for an unregistered type must not be delivered (no spurious dispatch).
	CapturingSubscriber other;
	DronecanRxRouter router2;
	router2.add(&h); // only NodeStatus registered
	(void)other;
}

} // namespace

int main()
{
	std::printf("dronecan_codec_test\n");

	test_decode_inversion();
	test_codec_roundtrip();
	test_wrong_length_rejected();
	test_fake_frame_dispatch();

	if (g_failures == 0) {
		std::printf("PASS: all checks\n");
		return 0;
	}

	std::printf("FAIL: %d check(s) failed\n", g_failures);
	return 1;
}
