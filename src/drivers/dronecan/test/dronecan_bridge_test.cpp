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
 * Off-target field-for-field contract test for the GNSS bridge mapping
 * (ACCEPTANCE_SPEC §1). Two layers:
 *   1. direct unit tests of every quirk pure function (fix2 fix-type remap, the
 *      INTENTIONAL 21->36 covariance FALLTHROUGH, the square-matrix unpack, the
 *      microsecond leap-second math, the Auxiliary freshness window, device id);
 *   2. a full encode -> broadcast -> loopback media -> receive -> router -> subscriber
 *      -> codec-decode -> map_common path that asserts the resulting sensor_gps
 *      against hand-computed expected values, including the Fix-vs-Fix2 per-node dedup.
 *
 * The subscriber is a host twin of the on-target Gnss bridge: it runs the exact same
 * platform-free pure functions (GnssDecode) the bridge does, against the REAL
 * generated sensor_gps_s (a tiny stubbed uORB/uORB.h satisfies the topic header
 * without linking uORB). Build & run with test/run_bridge_test.sh.
 */

#include "../DroneCANCodec.hpp"
#include "../DronecanHandle.hpp"
#include "../DronecanRxRouter.hpp"
#include "../RxSubscriberBase.hpp"
#include "../bridges/DeviceId.hpp"
#include "../bridges/GnssDecode.hpp"
#include "FakeCanardInterface.hpp"

#include <lib/mathlib/math/Limits.hpp>

#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.gnss.Fix.h>
#include <uavcan.equipment.gnss.Fix2.h>
#include <ardupilot.gnss.RelPosHeading.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace dronecan_gnss;

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

bool approx(float a, float b, float tol = 1e-4f)
{
	return std::fabs(a - b) <= tol;
}

constexpr uint8_t DEVTYPE_GPS = 0x85; // DRV_GPS_DEVTYPE_UAVCAN

// ---- Encode/decode wrappers (mirror the bridge's typed wrappers) -----------
uint32_t enc_fix(void *m, uint8_t *b, bool tao) { return uavcan_equipment_gnss_Fix_encode((uavcan_equipment_gnss_Fix *)m, b, tao); }
uint32_t enc_fix2(void *m, uint8_t *b, bool tao) { return uavcan_equipment_gnss_Fix2_encode((uavcan_equipment_gnss_Fix2 *)m, b, tao); }
uint32_t enc_aux(void *m, uint8_t *b, bool tao) { return uavcan_equipment_gnss_Auxiliary_encode((uavcan_equipment_gnss_Auxiliary *)m, b, tao); }
uint32_t enc_relpos(void *m, uint8_t *b, bool tao) { return ardupilot_gnss_RelPosHeading_encode((ardupilot_gnss_RelPosHeading *)m, b, tao); }

bool dec_fix(const CanardRxTransfer *t, void *m) { return uavcan_equipment_gnss_Fix_decode(t, (uavcan_equipment_gnss_Fix *)m); }
bool dec_fix2(const CanardRxTransfer *t, void *m) { return uavcan_equipment_gnss_Fix2_decode(t, (uavcan_equipment_gnss_Fix2 *)m); }
bool dec_aux(const CanardRxTransfer *t, void *m) { return uavcan_equipment_gnss_Auxiliary_decode(t, (uavcan_equipment_gnss_Auxiliary *)m); }
bool dec_relpos(const CanardRxTransfer *t, void *m) { return ardupilot_gnss_RelPosHeading_decode(t, (ardupilot_gnss_RelPosHeading *)m); }

const TypeDescriptor FixDesc  = {UAVCAN_EQUIPMENT_GNSS_FIX_ID, UAVCAN_EQUIPMENT_GNSS_FIX_SIGNATURE, UAVCAN_EQUIPMENT_GNSS_FIX_MAX_SIZE, CanardTransferTypeBroadcast, "Fix"};
const TypeDescriptor Fix2Desc = {UAVCAN_EQUIPMENT_GNSS_FIX2_ID, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE, UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE, CanardTransferTypeBroadcast, "Fix2"};
const TypeDescriptor AuxDesc  = {UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE, CanardTransferTypeBroadcast, "Aux"};
const TypeDescriptor RelDesc  = {ARDUPILOT_GNSS_RELPOSHEADING_ID, ARDUPILOT_GNSS_RELPOSHEADING_SIGNATURE, ARDUPILOT_GNSS_RELPOSHEADING_MAX_SIZE, CanardTransferTypeBroadcast, "Rel"};

/**
 * Host twin of the Gnss bridge: same pure functions, same control flow as
 * Gnss::handle_* + process_and_publish, minus hrt/uORB (publish records the report;
 * the clock side effect is omitted). A test-controlled `now_us` drives freshness.
 */
class GnssTwin : public RxSubscriberBase
{
public:
	uint64_t now_us{0};

	sensor_gps_s last{};
	int last_node{-1};
	int publish_count{0};

	void handle(const CanardRxTransfer &t, uint8_t iface_id) override
	{
		switch (t.data_type_id) {
		case UAVCAN_EQUIPMENT_GNSS_FIX_ID:  handle_fix(t, iface_id); break;

		case UAVCAN_EQUIPMENT_GNSS_FIX2_ID: handle_fix2(t, iface_id); break;

		case UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID: handle_aux(t); break;

		case ARDUPILOT_GNSS_RELPOSHEADING_ID: handle_relpos(t); break;

		default: break;
		}
	}

private:
	// Cached Auxiliary dop + RelPosHeading + per-channel dedup (bridge state).
	uint64_t _aux_ts{0};
	float _aux_hdop{0.f};
	float _aux_vdop{0.f};
	float _rel_heading{NAN};
	float _rel_heading_accuracy{NAN};
	bool _rel_heading_valid{false};
	int _node_ids[4] {-1, -1, -1, -1};
	bool _using_fix2[4] {};

	int8_t channel_index(int node) const
	{
		for (int i = 0; i < 4; i++) { if (_node_ids[i] == node) { return (int8_t)i; } }

		return -1;
	}

	void publish(int node, const sensor_gps_s &g)
	{
		if (channel_index(node) < 0) {
			for (int i = 0; i < 4; i++) { if (_node_ids[i] < 0) { _node_ids[i] = node; break; } }
		}

		last = g;
		last_node = node;
		publish_count++;
	}

	void handle_aux(const CanardRxTransfer &t)
	{
		uavcan_equipment_gnss_Auxiliary m{};

		if (!dronecan_codec::decodeTransfer(t, &m, dec_aux)) { return; }

		_aux_ts = now_us;
		_aux_hdop = m.hdop;
		_aux_vdop = m.vdop;
	}

	void handle_relpos(const CanardRxTransfer &t)
	{
		ardupilot_gnss_RelPosHeading m{};

		if (!dronecan_codec::decodeTransfer(t, &m, dec_relpos)) { return; }

		_rel_heading_valid = m.reported_heading_acc_available;
		_rel_heading = math::radians(m.reported_heading_deg);
		_rel_heading_accuracy = math::radians(m.reported_heading_acc_deg);
	}

	void handle_fix(const CanardRxTransfer &t, uint8_t iface_id)
	{
		uavcan_equipment_gnss_Fix m{};

		if (!dronecan_codec::decodeTransfer(t, &m, dec_fix)) { return; }

		const int node = t.source_node_id;
		const int8_t ch = channel_index(node);

		if (ch > -1 && _using_fix2[ch]) { return; }

		const uint8_t fix_type = m.status;
		const bool vpc = (m.position_covariance.len != 0);
		const bool vvc = (m.velocity_covariance.len != 0);
		float pos_cov[9];
		float vel_cov[9];
		unpack_square_matrix_3x3(m.position_covariance.data, m.position_covariance.len, pos_cov);
		unpack_square_matrix_3x3(m.velocity_covariance.data, m.velocity_covariance.len, vel_cov);
		process(m, node, iface_id, fix_type, pos_cov, vel_cov, vpc, vvc, NAN, NAN, NAN, -1, -1, 0, 0);
	}

	void handle_fix2(const CanardRxTransfer &t, uint8_t iface_id)
	{
		uavcan_equipment_gnss_Fix2 m{};

		if (!dronecan_codec::decodeTransfer(t, &m, dec_fix2)) { return; }

		const int node = t.source_node_id;
		const int8_t ch = channel_index(node);

		if (ch > -1 && !_using_fix2[ch]) { _using_fix2[ch] = true; }

		const uint8_t fix_type = fix2_remap_fix_type(m.status, m.mode, m.sub_mode);
		float pos_cov[9];
		float vel_cov[9];
		const bool valid = fix2_unpack_covariance(m, pos_cov, vel_cov);

		float heading = NAN, heading_offset = NAN, heading_accuracy = NAN;
		int32_t noise_per_ms = -1, jamming_indicator = -1;
		uint8_t jamming_state = 0, spoofing_state = 0;

		if (m.ecef_position_velocity.len != 0 && !_rel_heading_valid) {
			const auto &e = m.ecef_position_velocity.data[0];

			if (!std::isnan(e.velocity_xyz[0])) { heading = e.velocity_xyz[0]; }

			if (!std::isnan(e.velocity_xyz[1])) { heading_offset = e.velocity_xyz[1]; }

			if (!std::isnan(e.velocity_xyz[2])) { heading_accuracy = e.velocity_xyz[2]; }

			noise_per_ms = e.position_xyz_mm[0];
			jamming_indicator = e.position_xyz_mm[1];
			jamming_state = e.position_xyz_mm[2] >> 8;
			spoofing_state = e.position_xyz_mm[2] & 0xFF;
		}

		process(m, node, iface_id, fix_type, pos_cov, vel_cov, valid, valid, heading, heading_offset,
			heading_accuracy, noise_per_ms, jamming_indicator, jamming_state, spoofing_state);
	}

	template <typename FixT>
	void process(const FixT &m, int node, uint8_t iface_id, uint8_t fix_type,
		     const float (&pos_cov)[9], const float (&vel_cov)[9], bool vpc, bool vvc,
		     float heading, float heading_offset, float heading_accuracy,
		     int32_t noise_per_ms, int32_t jamming_indicator, uint8_t jamming_state, uint8_t spoofing_state)
	{
		sensor_gps_s g{};
		g.device_id = dronecan_bridge::make_uavcan_device_id((uint8_t)node, iface_id, DEVTYPE_GPS);
		g.timestamp = now_us;

		float h = heading, off = heading_offset, acc = heading_accuracy;

		if (_rel_heading_valid) {
			h = _rel_heading;
			off = NAN;
			acc = _rel_heading_accuracy;
			_rel_heading = NAN;
			_rel_heading_accuracy = NAN;
			_rel_heading_valid = false;
		}

		const bool aux_fresh = aux_is_fresh(now_us, _aux_ts);
		map_common(m, fix_type, pos_cov, vel_cov, vpc, vvc, h, off, acc, noise_per_ms,
			   jamming_indicator, jamming_state, spoofing_state, aux_fresh, _aux_hdop, _aux_vdop, g);

		publish(node, g);
	}
};

// ---- transport plumbing ----------------------------------------------------
struct Bus {
	FakeCanardInterface fake;
	DronecanRxRouter router;
	DronecanHandle handle{fake, router};
	RxHandler h_fix{}, h_fix2{}, h_aux{}, h_rel{};

	void start(GnssTwin &twin, uint8_t node_id)
	{
		handle.init();
		handle.setNodeID(node_id);
		h_fix  = {UAVCAN_EQUIPMENT_GNSS_FIX_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_FIX_SIGNATURE, &twin, nullptr};
		h_fix2 = {UAVCAN_EQUIPMENT_GNSS_FIX2_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE, &twin, nullptr};
		h_aux  = {UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE, &twin, nullptr};
		h_rel  = {ARDUPILOT_GNSS_RELPOSHEADING_ID, CanardTransferTypeBroadcast, ARDUPILOT_GNSS_RELPOSHEADING_SIGNATURE, &twin, nullptr};
		router.add(&h_fix);
		router.add(&h_fix2);
		router.add(&h_aux);
		router.add(&h_rel);
	}

	// Encode + broadcast + loopback (on the given iface) + receive: the full RX path
	// for one message. iface_id tags the looped-back frames so the bridge sees the
	// interface the transfer "arrived" on.
	template <typename T>
	void send(const TypeDescriptor &desc, T &msg, DroneCANEncodeFn enc, uint8_t &tid, uint8_t iface_id = 0)
	{
		uint8_t buf[256] {};
		const int32_t frames = dronecan_codec::encodeBroadcast(handle, desc, &msg, buf, &tid,
				       CANARD_TRANSFER_PRIORITY_MEDIUM, enc, 1000000);
		(void)frames;
		handle.transmit();
		fake.loopbackTxToRx(5000, iface_id);
		handle.receive();
		fake.clear();
	}
};

// ---- 1. quirk pure functions ----------------------------------------------
void test_fix2_remap()
{
	std::printf("test_fix2_remap\n");
	using F = uavcan_equipment_gnss_Fix2;
	// Non-differential modes pass status through.
	CHECK(fix2_remap_fix_type(3, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE, 0) == 3);
	CHECK(fix2_remap_fix_type(2, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_PPP, 0) == 2);
	// DGPS -> 4 regardless of status.
	CHECK(fix2_remap_fix_type(3, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS, 0) == 4);
	// RTK float / fixed.
	CHECK(fix2_remap_fix_type(3, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK, UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT) == 5);
	CHECK(fix2_remap_fix_type(3, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK, UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED) == 6);
	// RTK with an unknown sub-mode falls back to status.
	CHECK(fix2_remap_fix_type(3, UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK, 7) == 3);
	(void)sizeof(F);
}

void test_fix2_covariance_fallthrough()
{
	std::printf("test_fix2_covariance_fallthrough\n");

	// Size 6: diagonal.
	{
		uavcan_equipment_gnss_Fix2 m{};
		m.covariance.len = 6;
		m.covariance.data[0] = 1; m.covariance.data[1] = 2; m.covariance.data[2] = 3;
		m.covariance.data[3] = 4; m.covariance.data[4] = 5; m.covariance.data[5] = 6;
		float pc[9], vc[9];
		CHECK(fix2_unpack_covariance(m, pc, vc) == true);
		CHECK(pc[0] == 1 && pc[4] == 2 && pc[8] == 3);
		CHECK(vc[0] == 4 && vc[4] == 5 && vc[8] == 6);
		CHECK(pc[1] == 0 && pc[3] == 0); // off-diagonal stays zero
	}

	// Size 21 MUST fall through into the size-36 block (gnss.cpp:277), which then
	// falls through AGAIN into default (gnss.cpp:309) -- so valid_covariances ends
	// FALSE even though pos/vel cov were written with the size-36 mapping. This
	// double fallthrough is a stronger latent bug than the spec's note captured;
	// the port replicates it byte-for-byte. Build a buffer where the size-21 and
	// size-36 blocks pick DIFFERENT indices and assert the size-36 indices win.
	{
		uavcan_equipment_gnss_Fix2 m{};
		m.covariance.len = 21;

		for (int i = 0; i < 36; i++) { m.covariance.data[i] = (float)i; }

		float pc[9], vc[9];
		CHECK(fix2_unpack_covariance(m, pc, vc) == false); // 36 -> default -> invalid
		// Size-36 mapping: pos_cov from {0,1,2,6,7,8,12,13,14}, vel_cov from
		// {21,22,23,27,28,29,33,34,35}. (Size-21 would have used 0,1,2,1,6,7,...)
		CHECK(pc[0] == 0 && pc[3] == 6 && pc[8] == 14);
		CHECK(vc[0] == 21 && vc[4] == 28 && vc[8] == 35);
		// If the 21->36 fallthrough were absent, pc[3] would be covariance[1] == 1, not 6.
		CHECK(pc[3] != m.covariance.data[1]);
	}

	// Size 36: full 6x6 written, then 36 -> default -> valid_covariances false.
	{
		uavcan_equipment_gnss_Fix2 m{};
		m.covariance.len = 36;

		for (int i = 0; i < 36; i++) { m.covariance.data[i] = (float)i; }

		float pc[9], vc[9];
		CHECK(fix2_unpack_covariance(m, pc, vc) == false); // 36 -> default -> invalid
		CHECK(pc[0] == 0 && pc[3] == 6 && pc[8] == 14);
		CHECK(vc[0] == 21 && vc[8] == 35);
	}

	// Invalid size -> not valid, zeroed.
	{
		uavcan_equipment_gnss_Fix2 m{};
		m.covariance.len = 7;
		float pc[9], vc[9];
		CHECK(fix2_unpack_covariance(m, pc, vc) == false);
		CHECK(pc[0] == 0 && vc[0] == 0);
	}
}

void test_unpack_square_matrix()
{
	std::printf("test_unpack_square_matrix\n");
	float dst[9];

	const float scalar[1] = {5.f};
	unpack_square_matrix_3x3(scalar, 1, dst);
	CHECK(dst[0] == 5 && dst[4] == 5 && dst[8] == 5 && dst[1] == 0 && dst[5] == 0);

	const float diag[3] = {1.f, 2.f, 3.f};
	unpack_square_matrix_3x3(diag, 3, dst);
	CHECK(dst[0] == 1 && dst[4] == 2 && dst[8] == 3 && dst[1] == 0 && dst[2] == 0);

	// Symmetric (upper triangle a..f): rows [a b c][b d e][c e f].
	const float sym[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	unpack_square_matrix_3x3(sym, 6, dst);
	CHECK(dst[0] == 1 && dst[1] == 2 && dst[2] == 3);
	CHECK(dst[3] == 2 && dst[4] == 4 && dst[5] == 5);
	CHECK(dst[6] == 3 && dst[7] == 5 && dst[8] == 6);

	float full[9];

	for (int i = 0; i < 9; i++) { full[i] = (float)(i + 1); }

	unpack_square_matrix_3x3(full, 9, dst);

	for (int i = 0; i < 9; i++) { CHECK(dst[i] == (float)(i + 1)); }

	const float junk[5] = {9, 9, 9, 9, 9};
	unpack_square_matrix_3x3(junk, 5, dst);

	for (int i = 0; i < 9; i++) { CHECK(dst[i] == 0); }
}

void test_leap_seconds()
{
	std::printf("test_leap_seconds\n");
	const uint64_t ts = 1000000000ULL;

	// UTC: as-is.
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_UTC, ts, 18) == ts);
	// GPS, leap>0: ts - leap + 9 (applied in microseconds, verbatim).
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_GPS, ts, 18) == ts - 18 + 9);
	// TAI, leap>0: ts - leap - 10.
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_TAI, ts, 18) == ts - 18 - 10);
	// GPS/TAI with leap UNKNOWN(0): left 0.
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_GPS, ts, 0) == 0);
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_TAI, ts, 0) == 0);
	// NONE: 0.
	CHECK(leap_corrected_utc_usec(GNSS_TIME_STANDARD_NONE, ts, 18) == 0);
}

void test_aux_freshness()
{
	std::printf("test_aux_freshness\n");
	CHECK(aux_is_fresh(1000000, 1000000) == true);    // same instant
	CHECK(aux_is_fresh(1000000 + 1999999, 1000000) == true);  // < 2 s
	CHECK(aux_is_fresh(1000000 + 2000000, 1000000) == false); // == 2 s
	CHECK(aux_is_fresh(1000000 + 5000000, 1000000) == false); // > 2 s
	CHECK(aux_is_fresh(0, 0) == true);
}

void test_device_id()
{
	std::printf("test_device_id\n");
	// ACCEPTANCE_SPEC §2 example: node 125 on CAN1, devtype 0x85 -> 0x00857D03.
	CHECK(dronecan_bridge::make_uavcan_device_id(125, 0, 0x85) == 0x00857D03u);
	// CAN2 sets the bus nibble: node 42 on iface 1 -> 0x00852A0B.
	CHECK(dronecan_bridge::make_uavcan_device_id(42, 1, 0x85) == 0x00852A0Bu);
}

// ---- 2. full path: Auxiliary then Fix2 -> sensor_gps field-for-field --------
void test_full_path_fix2()
{
	std::printf("test_full_path_fix2\n");
	GnssTwin twin;
	twin.now_us = 100000000ULL;
	Bus bus;
	bus.start(twin, 42);

	uint8_t tid_aux = 0, tid_fix2 = 0;

	// Auxiliary first: hdop/vdop become the fresh source for the following Fix2.
	uavcan_equipment_gnss_Auxiliary aux{};
	aux.hdop = 1.5f;     // float16-exact
	aux.vdop = 2.5f;     // float16-exact
	bus.send(AuxDesc, aux, enc_aux, tid_aux);

	uavcan_equipment_gnss_Fix2 fix2{};
	fix2.latitude_deg_1e8  = 4700000000LL;   // 47.0 deg
	fix2.longitude_deg_1e8 = 800000000LL;    // 8.0 deg
	fix2.height_msl_mm     = 500000;         // 500 m
	fix2.height_ellipsoid_mm = 510000;       // 510 m
	fix2.ned_velocity[0] = 1.0f;
	fix2.ned_velocity[1] = 2.0f;
	fix2.ned_velocity[2] = -0.5f;
	fix2.sats_used = 12;
	fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
	fix2.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
	fix2.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED; // -> fix_type 6
	fix2.covariance.len = 6;                 // diagonal, float16-exact values
	fix2.covariance.data[0] = 0.25f;
	fix2.covariance.data[1] = 0.25f;
	fix2.covariance.data[2] = 0.5f;
	fix2.covariance.data[3] = 0.0625f;
	fix2.covariance.data[4] = 0.0625f;
	fix2.covariance.data[5] = 0.0625f;
	fix2.pdop = 2.0f;
	fix2.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
	fix2.gnss_timestamp.usec = 1234567890123456ULL;
	fix2.num_leap_seconds = 18;
	fix2.ecef_position_velocity.len = 0;     // no heading hack

	bus.send(Fix2Desc, fix2, enc_fix2, tid_fix2);

	CHECK(twin.publish_count == 1);
	CHECK(twin.last_node == 42);

	const sensor_gps_s &g = twin.last;

	// Identity / integer-derived fields: exact.
	CHECK(g.device_id == dronecan_bridge::make_uavcan_device_id(42, 0, DEVTYPE_GPS));
	CHECK(g.latitude_deg == 47.0);
	CHECK(g.longitude_deg == 8.0);
	CHECK(g.altitude_msl_m == 500.0);
	CHECK(g.altitude_ellipsoid_m == 510.0);
	CHECK(g.fix_type == 6);
	CHECK(g.satellites_used == 12);
	CHECK(g.vel_ned_valid == true);
	CHECK(g.timestamp_time_relative == 0);
	CHECK(g.time_utc_usec == 1234567890123456ULL);
	CHECK(g.vel_n_m_s == 1.0f);
	CHECK(g.vel_e_m_s == 2.0f);
	CHECK(g.vel_d_m_s == -0.5f);
	CHECK(g.hdop == 1.5f);     // fresh Auxiliary
	CHECK(g.vdop == 2.5f);
	CHECK(g.noise_per_ms == -1);
	CHECK(g.jamming_indicator == -1);
	CHECK(g.jamming_state == 0);
	CHECK(g.spoofing_state == 0);
	CHECK(std::isnan(g.heading));
	CHECK(std::isnan(g.heading_offset));
	CHECK(std::isnan(g.heading_accuracy));

	// Computed fields: tolerance (float16 covariance / sqrt / atan2).
	CHECK(approx(g.eph, 0.5f));               // sqrt(max(0.25,0.25))
	CHECK(approx(g.epv, sqrtf(0.5f)));        // sqrt(0.5)
	CHECK(approx(g.s_variance_m_s, 0.0625f)); // max of diagonal vel cov
	CHECK(approx(g.c_variance_rad, 0.0125f)); // Jacobian, off-diagonal vel cov == 0
	CHECK(approx(g.vel_m_s, sqrtf(5.25f)));
	CHECK(approx(g.cog_rad, atan2f(2.0f, 1.0f)));
}

// ---- 3. Fix-vs-Fix2 per-node dedup -----------------------------------------
void test_fix_fix2_dedup()
{
	std::printf("test_fix_fix2_dedup\n");
	GnssTwin twin;
	twin.now_us = 100000000ULL;
	Bus bus;
	bus.start(twin, 42);

	uint8_t tid_fix2 = 0, tid_fix = 0;

	uavcan_equipment_gnss_Fix2 fix2{};
	fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
	fix2.covariance.len = 0;

	// Fix2 #1: no channel yet -> flag NOT latched, but publish creates the channel.
	bus.send(Fix2Desc, fix2, enc_fix2, tid_fix2);
	CHECK(twin.publish_count == 1);

	// Fix2 #2: channel exists -> flag latches true.
	bus.send(Fix2Desc, fix2, enc_fix2, tid_fix2);
	CHECK(twin.publish_count == 2);

	// Fix from the same node is now dropped (channel flagged Fix2).
	uavcan_equipment_gnss_Fix fix{};
	fix.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_3D_FIX;
	fix.position_covariance.len = 0;
	fix.velocity_covariance.len = 0;
	bus.send(FixDesc, fix, enc_fix, tid_fix);
	CHECK(twin.publish_count == 2); // unchanged: Fix ignored
}

// ---- 4. RelPosHeading -> heading (deg->rad), consumed once ------------------
void test_relpos_heading()
{
	std::printf("test_relpos_heading\n");
	GnssTwin twin;
	twin.now_us = 100000000ULL;
	Bus bus;
	bus.start(twin, 7);

	uint8_t tid_rel = 0, tid_fix2 = 0;

	ardupilot_gnss_RelPosHeading rel{};
	rel.reported_heading_acc_available = true;
	rel.reported_heading_deg = 90.0f;
	rel.reported_heading_acc_deg = 1.0f;
	bus.send(RelDesc, rel, enc_relpos, tid_rel);

	uavcan_equipment_gnss_Fix2 fix2{};
	fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
	fix2.covariance.len = 0;
	fix2.ecef_position_velocity.len = 0;

	// First Fix2 after RelPosHeading: heading wins, offset NAN, then consumed.
	bus.send(Fix2Desc, fix2, enc_fix2, tid_fix2);
	CHECK(approx(twin.last.heading, math::radians(90.0f)));
	CHECK(std::isnan(twin.last.heading_offset));
	CHECK(approx(twin.last.heading_accuracy, math::radians(1.0f)));

	// Second Fix2 without a fresh RelPosHeading: heading reverts to NAN (no ecef hack).
	bus.send(Fix2Desc, fix2, enc_fix2, tid_fix2);
	CHECK(std::isnan(twin.last.heading));
	CHECK(std::isnan(twin.last.heading_accuracy));
}

// ---- 5. CAN interface -> sensor_gps device_id bus field --------------------
void test_iface_routing()
{
	std::printf("test_iface_routing\n");

	uavcan_equipment_gnss_Fix2 fix2{};
	fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
	fix2.covariance.len = 0;
	fix2.ecef_position_velocity.len = 0;

	// Same node, delivered on CAN1 (iface 0) and CAN2 (iface 1): the device_id bus
	// field must follow the interface the transfer arrived on.
	{
		GnssTwin twin;
		twin.now_us = 100000000ULL;
		Bus bus;
		bus.start(twin, 9);
		uint8_t tid = 0;
		bus.send(Fix2Desc, fix2, enc_fix2, tid, /*iface*/ 0);
		CHECK(twin.last.device_id == dronecan_bridge::make_uavcan_device_id(9, 0, DEVTYPE_GPS));
		CHECK(((twin.last.device_id >> 3) & 0x1F) == 0); // bus = CAN1
	}
	{
		GnssTwin twin;
		twin.now_us = 100000000ULL;
		Bus bus;
		bus.start(twin, 9);
		uint8_t tid = 0;
		bus.send(Fix2Desc, fix2, enc_fix2, tid, /*iface*/ 1);
		CHECK(twin.last.device_id == dronecan_bridge::make_uavcan_device_id(9, 1, DEVTYPE_GPS));
		CHECK(((twin.last.device_id >> 3) & 0x1F) == 1); // bus = CAN2
	}
}

} // namespace

int main()
{
	std::printf("dronecan_bridge_test\n");

	test_fix2_remap();
	test_fix2_covariance_fallthrough();
	test_unpack_square_matrix();
	test_leap_seconds();
	test_aux_freshness();
	test_device_id();
	test_full_path_fix2();
	test_fix_fix2_dedup();
	test_relpos_heading();
	test_iface_routing();

	if (g_failures == 0) {
		std::printf("PASS: all checks\n");
		return 0;
	}

	std::printf("FAIL: %d check(s) failed\n", g_failures);
	return 1;
}
