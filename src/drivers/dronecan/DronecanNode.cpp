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

#include "DronecanNode.hpp"

#include <lib/parameters/param.h>

#include <canard.h>

#include <inttypes.h>
#include <string.h>

using namespace time_literals;

DronecanNode *DronecanNode::_instance;

namespace
{
// P4.0 link-proof trampolines: minimal callbacks matching the libcanard v0
// signatures so canardInit() can be exercised below. The real
// DronecanHandle (P4.1) carries its own shouldAccept/onReception trampolines;
// the node adopts the handle and drops this placeholder in P4.3.
bool dc_should_accept(const CanardInstance *ins, uint64_t *out_sig, uint16_t dtid,
		      CanardTransferType transfer_type, uint8_t source_node_id)
{
	(void)ins;
	(void)dtid;
	(void)transfer_type;
	(void)source_node_id;

	if (out_sig != nullptr) {
		*out_sig = 0;
	}

	return false;
}

void dc_on_reception(CanardInstance *ins, CanardRxTransfer *transfer)
{
	(void)ins;
	(void)transfer;
}
} // namespace

DronecanNode::DronecanNode(uint32_t node_id, uint32_t bitrate) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_node_id(node_id),
	_bitrate(bitrate)
{
	pthread_mutex_init(&_node_mutex, nullptr);
}

DronecanNode::~DronecanNode()
{
	ScheduleClear();

	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	pthread_mutex_destroy(&_node_mutex);

	_instance = nullptr;
}

int DronecanNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	_instance = new DronecanNode(node_id, bitrate);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void DronecanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	perf_count(_interval_perf);
	perf_begin(_cycle_perf);

	// P4.3: lazy CAN bring-up, then transmit(); receive(); transmit(); under the
	// node mutex, driving the DronecanHandle. The P4.0 scaffold has no transport yet.

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);
}

void DronecanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	PX4_INFO("DroneCAN node id %" PRIu32 ", bitrate %" PRIu32, _node_id, _bitrate);

	// P4.0 link-proof: exercise libcanard so canard.c is genuinely linked, not just
	// compiled. Replaced by DronecanHandle ownership of the CanardInstance in P4.3.
	static uint8_t pool[CANARD_MEM_BLOCK_SIZE * 4];
	CanardInstance ins{};
	canardInit(&ins, pool, sizeof(pool), &dc_on_reception, &dc_should_accept, nullptr);
	PX4_INFO("libcanard block size %u B, local node id %u",
		 (unsigned)CANARD_MEM_BLOCK_SIZE, (unsigned)canardGetLocalNodeID(&ins));

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: dronecan {start|status|stop}");
}

extern "C" __EXPORT int dronecan_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (DronecanNode::instance() != nullptr) {
			PX4_ERR("already started");
			return 1;
		}

		int32_t node_id = 0;
		param_get(param_find("DC_NODE_ID"), &node_id);

		int32_t bitrate = 0;
		param_get(param_find("DC_BITRATE"), &bitrate);

		PX4_INFO("Node ID %" PRId32 ", bitrate %" PRId32, node_id, bitrate);
		return DronecanNode::start((uint32_t)node_id, (uint32_t)bitrate);
	}

	/* commands below require the app to be started */
	DronecanNode *const inst = DronecanNode::instance();

	if (inst == nullptr) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
