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

#include "SensorBridgeBase.hpp"

#include <px4_platform_common/log.h>

SensorBridgeBase::~SensorBridgeBase()
{
	for (auto &ch : _channels) {
		if (ch.orb_advert != nullptr) {
			orb_unadvertise(ch.orb_advert);
			ch.orb_advert = nullptr;
		}
	}
}

void SensorBridgeBase::publish(const int node_id, const void *report)
{
	Channel *channel = nullptr;

	// Existing channel?
	for (auto &ch : _channels) {
		if (ch.node_id == node_id) {
			channel = &ch;
			break;
		}
	}

	// No channel yet -- allocate one.
	if (channel == nullptr) {
		if (_out_of_channels) {
			return;  // Give up immediately -- saves some CPU time.
		}

		for (auto &ch : _channels) {
			if (ch.node_id < 0) {
				channel = &ch;
				break;
			}
		}

		if (channel == nullptr) {
			_out_of_channels = true;
			PX4_DEBUG("%s: out of channels", _name);
			return;
		}

		channel->orb_advert = orb_advertise_multi(_orb_topic, report, &channel->instance);
		channel->node_id = node_id;

		if (channel->orb_advert == nullptr) {
			PX4_DEBUG("%s: uORB advertise failed", _name);
			*channel = Channel();
			_out_of_channels = true;
			return;
		}

		PX4_DEBUG("%s: node %d -> instance %d", _name, channel->node_id, channel->instance);
	}

	(void)orb_publish(_orb_topic, channel->orb_advert, report);
}

unsigned SensorBridgeBase::get_num_redundant_channels() const
{
	unsigned out = 0;

	for (const auto &ch : _channels) {
		if (ch.node_id >= 0) {
			out++;
		}
	}

	return out;
}

int8_t SensorBridgeBase::get_channel_index_for_node(int node_id) const
{
	for (unsigned i = 0; i < DEFAULT_MAX_CHANNELS; i++) {
		if (_channels[i].node_id == node_id) {
			return (int8_t)i;
		}
	}

	return -1;
}

void SensorBridgeBase::print_status() const
{
	PX4_INFO("%s: %u channel(s)", _name, get_num_redundant_channels());

	for (unsigned i = 0; i < DEFAULT_MAX_CHANNELS; i++) {
		if (_channels[i].node_id >= 0) {
			PX4_INFO("  channel %u: node id %d --> instance %d",
				 i, _channels[i].node_id, _channels[i].instance);
		}
	}
}
