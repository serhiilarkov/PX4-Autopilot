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

/**
 * @file GnssDecode.hpp
 *
 * Platform-free GNSS message -> sensor_gps field mapping for the dronecan GNSS
 * bridge. Every quirk the ACCEPTANCE_SPEC §1 calls out lives here as a named pure
 * function so it can be unit-tested off-target against the spec, byte-identically
 * to the libuavcan source (src/drivers/uavcan/sensors/gnss.cpp). The on-target
 * bridge (Gnss.cpp) supplies the stateful/platform parts (hrt timestamp, device id,
 * rel-heading consumption, system-clock side effect, publish).
 *
 * Depends only on the generated DSDL C structs and sensor_gps_s -- no uORB/hrt/perf.
 */

#pragma once

#include <uavcan.equipment.gnss.Fix.h>
#include <uavcan.equipment.gnss.Fix2.h>
#include <uORB/topics/sensor_gps.h>

#include <math.h>
#include <stdint.h>

namespace dronecan_gnss
{

// GNSS time standards. Values are identical across the Fix (1060) and Fix2 (1063)
// DSDL definitions, so a single set serves both code paths.
enum {
	GNSS_TIME_STANDARD_NONE = 0,
	GNSS_TIME_STANDARD_TAI  = 1,
	GNSS_TIME_STANDARD_UTC  = 2,
	GNSS_TIME_STANDARD_GPS  = 3,
};

// Auxiliary hdop/vdop freshness window (gnss.cpp:525): 2 s.
static constexpr uint64_t kAuxFreshnessUs = 2000000ULL;

/**
 * Fix2 mode/sub_mode -> sensor_gps fix_type remap (gnss.cpp:192-211). Starts from
 * msg.status (0..3) and upgrades for differential/RTK modes:
 *   MODE_DGPS                  -> 4 (RTCM code differential)
 *   MODE_RTK + SUB_MODE_FLOAT  -> 5 (RTK float)
 *   MODE_RTK + SUB_MODE_FIXED  -> 6 (RTK fixed)
 * Any other mode leaves fix_type == status.
 */
uint8_t fix2_remap_fix_type(uint8_t status, uint8_t mode, uint8_t sub_mode);

/**
 * Unpack Fix2 msg.covariance (float16[<=36]) into 3x3 pos/vel covariance matrices
 * (gnss.cpp:213-315). Returns valid_covariances. Replicates the hand-optimized
 * size-1/6/21/36 unpack INCLUDING the INTENTIONAL `case 21:` -> `case 36:`
 * FALLTHROUGH (gnss.cpp:277) -- a size-21 message also runs the size-36 block,
 * overwriting pos/vel cov. This is a latent bug; the port reproduces it byte-for-byte.
 */
bool fix2_unpack_covariance(const uavcan_equipment_gnss_Fix2 &msg, float (&pos_cov)[9], float (&vel_cov)[9]);

/**
 * Unpack a packed DroneCAN square-matrix array (float[<=9]) into a row-major 3x3
 * (the Fix path's position_covariance.unpackSquareMatrix, REF array.hpp:602):
 *   len 1     -> scalar on the diagonal
 *   len 3     -> diagonal
 *   len 6     -> symmetric (upper triangle, lower mirrored)
 *   len 9     -> full
 *   otherwise -> all zero
 */
void unpack_square_matrix_3x3(const float *src, uint8_t len, float (&dst)[9]);

/**
 * Apply the GNSS time-standard leap-second correction to a microsecond timestamp
 * (gnss.cpp:483-506). UTC: as-is. GPS: `ts - num_leap_seconds + 9` only if leap>0.
 * TAI: `ts - num_leap_seconds - 10` only if leap>0. NONE / leap==0 (UNKNOWN): 0.
 *
 * NB the +9 / -10 / -num_leap_seconds offsets are applied in MICROSECONDS to a
 * microsecond value -- a tiny/incorrect offset vs the seconds-scaled DSDL formula.
 * Replicated verbatim for parity (ACCEPTANCE_SPEC §1 "UTC subtlety").
 */
uint64_t leap_corrected_utc_usec(uint8_t time_standard, uint64_t gnss_ts_usec, uint8_t num_leap_seconds);

/// True if an Auxiliary message seen at last_aux_us is still fresh at now_us
/// (within the 2 s window, gnss.cpp:525). now_us < last_aux_us yields a huge
/// unsigned wrap == not fresh, matching hrt_elapsed_time on a future stamp.
bool aux_is_fresh(uint64_t now_us, uint64_t last_aux_us);

/**
 * Map the fields common to Fix and Fix2 into sensor_gps (the process_fixx body,
 * gnss.cpp:405-557). The caller pre-computes everything that is type- or
 * state-dependent (fix_type, covariances + validity, heading triple, jamming/noise)
 * and the freshness of the cached Auxiliary dop. Templated over the generated Fix /
 * Fix2 struct -- both expose the same scalar field names.
 *
 * Writes only the fields the libuavcan bridge sets in process_fixx; `out` must be
 * zero-initialized by the caller. Does NOT set timestamp (hrt), device_id,
 * selected_rtcm_instance or rtcm_injection_rate (bridge state), and performs no
 * side effects (the CLOCK_REALTIME set is left to the bridge).
 */
template <typename FixT>
void map_common(const FixT &msg,
		uint8_t fix_type,
		const float (&pos_cov)[9], const float (&vel_cov)[9],
		bool valid_pos_cov, bool valid_vel_cov,
		float heading, float heading_offset, float heading_accuracy,
		int32_t noise_per_ms, int32_t jamming_indicator,
		uint8_t jamming_state, uint8_t spoofing_state,
		bool aux_fresh, float aux_hdop, float aux_vdop,
		sensor_gps_s &out)
{
	out.latitude_deg         = msg.latitude_deg_1e8 / 1e8;
	out.longitude_deg        = msg.longitude_deg_1e8 / 1e8;
	out.altitude_msl_m       = msg.height_msl_mm / 1e3;
	out.altitude_ellipsoid_m = msg.height_ellipsoid_mm / 1e3;

	if (valid_pos_cov) {
		// Horizontal position uncertainty -- math::max(a,b) == (a > b) ? a : b.
		const float horizontal_pos_variance = (pos_cov[0] > pos_cov[4]) ? pos_cov[0] : pos_cov[4];
		out.eph = (horizontal_pos_variance > 0) ? sqrtf(horizontal_pos_variance) : -1.0F;
		out.epv = (pos_cov[8] > 0) ? sqrtf(pos_cov[8]) : -1.0F;

	} else {
		out.eph = -1.0F;
		out.epv = -1.0F;
	}

	if (valid_vel_cov) {
		const float a = (vel_cov[0] > vel_cov[4]) ? vel_cov[0] : vel_cov[4];
		out.s_variance_m_s = (a > vel_cov[8]) ? a : vel_cov[8];

		// Jacobian of heading = atan2(vel_e, vel_n) to map velocity covariance to
		// heading variance (gnss.cpp:448-465). Division by zero at zero velocity
		// yields inf/nan exactly as the source -- not guarded.
		const float vel_n = msg.ned_velocity[0];
		const float vel_e = msg.ned_velocity[1];
		const float vel_n_sq = vel_n * vel_n;
		const float vel_e_sq = vel_e * vel_e;
		out.c_variance_rad =
			(vel_e_sq * vel_cov[0] +
			 -2 * vel_n * vel_e * vel_cov[1] +
			 vel_n_sq * vel_cov[4]) / ((vel_n_sq + vel_e_sq) * (vel_n_sq + vel_e_sq));

	} else {
		out.s_variance_m_s = -1.0F;
		out.c_variance_rad = -1.0F;
	}

	out.fix_type = fix_type;

	out.vel_n_m_s = msg.ned_velocity[0];
	out.vel_e_m_s = msg.ned_velocity[1];
	out.vel_d_m_s = msg.ned_velocity[2];
	out.vel_m_s = sqrtf(msg.ned_velocity[0] * msg.ned_velocity[0]
			    + msg.ned_velocity[1] * msg.ned_velocity[1]
			    + msg.ned_velocity[2] * msg.ned_velocity[2]);
	out.cog_rad = atan2f(out.vel_e_m_s, out.vel_n_m_s);
	out.vel_ned_valid = true;

	out.timestamp_time_relative = 0;

	out.time_utc_usec = leap_corrected_utc_usec(msg.gnss_time_standard, msg.gnss_timestamp.usec,
			    msg.num_leap_seconds);

	out.satellites_used = msg.sats_used;

	if (aux_fresh) {
		out.hdop = aux_hdop;
		out.vdop = aux_vdop;

	} else {
		// PDOP used for both HDOP and VDOP (issue #5153).
		out.hdop = msg.pdop;
		out.vdop = msg.pdop;
	}

	out.heading = heading;
	out.heading_offset = heading_offset;
	out.heading_accuracy = heading_accuracy;

	out.noise_per_ms = noise_per_ms;
	out.jamming_indicator = jamming_indicator;
	out.jamming_state = jamming_state;
	out.spoofing_state = spoofing_state;
}

} // namespace dronecan_gnss
