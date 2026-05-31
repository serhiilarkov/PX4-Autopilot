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

#include "GnssDecode.hpp"

namespace dronecan_gnss
{

uint8_t fix2_remap_fix_type(uint8_t status, uint8_t mode, uint8_t sub_mode)
{
	uint8_t fix_type = status;

	switch (mode) {
	case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS:
		fix_type = 4; // RTCM code differential
		break;

	case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK:
		switch (sub_mode) {
		case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT:
			fix_type = 5; // RTK float
			break;

		case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED:
			fix_type = 6; // RTK fixed
			break;
		}

		break;
	}

	return fix_type;
}

bool fix2_unpack_covariance(const uavcan_equipment_gnss_Fix2 &msg, float (&pos_cov)[9], float (&vel_cov)[9])
{
	for (int i = 0; i < 9; i++) {
		pos_cov[i] = 0.f;
		vel_cov[i] = 0.f;
	}

	bool valid_covariances = true;

	switch (msg.covariance.len) {
	case 1: {
			// Scalar matrix
			const float x = msg.covariance.data[0];

			pos_cov[0] = x;
			pos_cov[4] = x;
			pos_cov[8] = x;

			vel_cov[0] = x;
			vel_cov[4] = x;
			vel_cov[8] = x;
		}
		break;

	case 6: {
			// Diagonal matrix (the most common case)
			pos_cov[0] = msg.covariance.data[0];
			pos_cov[4] = msg.covariance.data[1];
			pos_cov[8] = msg.covariance.data[2];

			vel_cov[0] = msg.covariance.data[3];
			vel_cov[4] = msg.covariance.data[4];
			vel_cov[8] = msg.covariance.data[5];

		}
		break;


	case 21: {
			// Upper triangular matrix.
			// Sub-matrix indexes (empty squares contain velocity-position covariance data):
			// 0  1  2
			// 1  6  7
			// 2  7 11
			//         15 16 17
			//         16 18 19
			//         17 19 20
			pos_cov[0] = msg.covariance.data[0];
			pos_cov[1] = msg.covariance.data[1];
			pos_cov[2] = msg.covariance.data[2];
			pos_cov[3] = msg.covariance.data[1];
			pos_cov[4] = msg.covariance.data[6];
			pos_cov[5] = msg.covariance.data[7];
			pos_cov[6] = msg.covariance.data[2];
			pos_cov[7] = msg.covariance.data[7];
			pos_cov[8] = msg.covariance.data[11];

			vel_cov[0] = msg.covariance.data[15];
			vel_cov[1] = msg.covariance.data[16];
			vel_cov[2] = msg.covariance.data[17];
			vel_cov[3] = msg.covariance.data[16];
			vel_cov[4] = msg.covariance.data[18];
			vel_cov[5] = msg.covariance.data[19];
			vel_cov[6] = msg.covariance.data[17];
			vel_cov[7] = msg.covariance.data[19];
			vel_cov[8] = msg.covariance.data[20];
		}

	/* FALLTHROUGH */
	case 36: {
			// Full matrix 6x6.
			// Sub-matrix indexes (empty squares contain velocity-position covariance data):
			//  0  1  2
			//  6  7  8
			// 12 13 14
			//          21 22 23
			//          27 28 29
			//          33 34 35
			pos_cov[0] = msg.covariance.data[0];
			pos_cov[1] = msg.covariance.data[1];
			pos_cov[2] = msg.covariance.data[2];
			pos_cov[3] = msg.covariance.data[6];
			pos_cov[4] = msg.covariance.data[7];
			pos_cov[5] = msg.covariance.data[8];
			pos_cov[6] = msg.covariance.data[12];
			pos_cov[7] = msg.covariance.data[13];
			pos_cov[8] = msg.covariance.data[14];

			vel_cov[0] = msg.covariance.data[21];
			vel_cov[1] = msg.covariance.data[22];
			vel_cov[2] = msg.covariance.data[23];
			vel_cov[3] = msg.covariance.data[27];
			vel_cov[4] = msg.covariance.data[28];
			vel_cov[5] = msg.covariance.data[29];
			vel_cov[6] = msg.covariance.data[33];
			vel_cov[7] = msg.covariance.data[34];
			vel_cov[8] = msg.covariance.data[35];
		}

	/* FALLTHROUGH */
	default: {
			// Either empty or invalid sized, interpret as zero matrix
			valid_covariances = false;
			break;	// Nothing to do
		}
	}

	return valid_covariances;
}

void unpack_square_matrix_3x3(const float *src, uint8_t len, float (&dst)[9])
{
	constexpr int N = 3;            // rows/cols
	constexpr int TRI = 6;          // elements in the upper triangle (incl. diagonal)

	if (len == N || len == 1) {     // Scalar or diagonal
		for (int index = 0; index < 9; index++) {
			const bool on_diagonal = (index / N) == (index % N);

			if (on_diagonal) {
				const int source_index = (len == 1) ? 0 : (index / N);
				dst[index] = src[source_index];

			} else {
				dst[index] = 0.f;
			}
		}

	} else if (len == TRI) {        // Symmetric (upper triangle, lower mirrored)
		int source_index = 0;

		for (int row = 0; row < N; row++) {
			for (int col = 0; col < N; col++) {
				if (col >= row) {
					dst[row * N + col] = src[source_index];
					source_index++;

				} else {
					// Transpose of an already-written upper-triangle element.
					dst[row * N + col] = dst[col * N + row];
				}
			}
		}

	} else if (len == 9) {          // Full
		for (int index = 0; index < 9; index++) {
			dst[index] = src[index];
		}

	} else {                        // Everything else -> zero
		for (int index = 0; index < 9; index++) {
			dst[index] = 0.f;
		}
	}
}

uint64_t leap_corrected_utc_usec(uint8_t time_standard, uint64_t gnss_ts_usec, uint8_t num_leap_seconds)
{
	uint64_t utc_usec = 0;

	switch (time_standard) {
	case GNSS_TIME_STANDARD_UTC:
		utc_usec = gnss_ts_usec;
		break;

	case GNSS_TIME_STANDARD_GPS:
		if (num_leap_seconds > 0) {
			utc_usec = gnss_ts_usec - num_leap_seconds + 9;
		}

		break;

	case GNSS_TIME_STANDARD_TAI:
		if (num_leap_seconds > 0) {
			utc_usec = gnss_ts_usec - num_leap_seconds - 10;
		}

		break;

	default:
		break;
	}

	return utc_usec;
}

bool aux_is_fresh(uint64_t now_us, uint64_t last_aux_us)
{
	return (now_us - last_aux_us) < kAuxFreshnessUs;
}

} // namespace dronecan_gnss
