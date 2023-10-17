/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file RGBController.hpp
 *
 * Defines basic functionality of Cyphal RGBController publisher
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <reg/udral/physics/optics/HighColor_0_1.h>
#include <lib/led/led.h>

#include "../Publisher.hpp"

class RGBControllerPublisher : public UavcanPublisher
{
public:
	RGBControllerPublisher(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(handle, pmgr, "udral.", "rgbled", instance)
	{

	};

	~RGBControllerPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		if (_port_id == CANARD_PORT_ID_UNSET) {
			return;
		}

		auto current_time_us = hrt_absolute_time();
		if (current_time_us < _next_pub_time_us) {
			return;
		}

		_next_pub_time_us = current_time_us + 50;

		LedControlData led_control_data;
		if(_led_controller.update(led_control_data) != 1) {
			return;
		}

		uint8_t brightness = led_control_data.leds[0].brightness;
		reg_udral_physics_optics_HighColor_0_1 msg;

		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			msg.red = brightness >> 3;
			msg.green = 0;
			msg.blue = 0;
			break;

		case led_control_s::COLOR_GREEN:
			msg.red = 0;
			msg.green = brightness >> 2;
			msg.blue = 0;
			break;

		case led_control_s::COLOR_BLUE:
			msg.red = 0;
			msg.green = 0;
			msg.blue = brightness >> 3;
			break;

		case led_control_s::COLOR_AMBER: // make it the same as yellow

		// FALLTHROUGH
		case led_control_s::COLOR_YELLOW:
			msg.red = (brightness / 2) >> 3;
			msg.green = (brightness / 2) >> 2;
			msg.blue = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			msg.red = (brightness / 2) >> 3;
			msg.green = 0;
			msg.blue = (brightness / 2) >> 3;
			break;

		case led_control_s::COLOR_CYAN:
			msg.red = 0;
			msg.green = (brightness / 2) >> 2;
			msg.blue = (brightness / 2) >> 3;
			break;

		case led_control_s::COLOR_WHITE:
			msg.red = (brightness / 3) >> 3;
			msg.green = (brightness / 3) >> 2;
			msg.blue = (brightness / 3) >> 3;
			break;

		default: // led_control_s::COLOR_OFF
			msg.red = 0;
			msg.green = 0;
			msg.blue = 0;
			break;
		}

		msg.red /= 8;
		msg.green /= 8;
		msg.blue /= 8;

		const CanardTransferMetadata transfer_metadata = {
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = _port_id,
			.remote_node_id = CANARD_NODE_ID_UNSET,
			.transfer_id    = _transfer_id,
		};

		uint8_t payload[reg_udral_physics_optics_HighColor_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t payload_size = reg_udral_physics_optics_HighColor_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
		int32_t result = reg_udral_physics_optics_HighColor_0_1_serialize_(&msg, payload, &payload_size);
		if (NUNAVUT_SUCCESS == result) {
			++_transfer_id;
			_canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						&transfer_metadata,
						payload_size,
						&payload);
		}
	};

private:

	LedController _led_controller;
	CanardTransferID _transfer_id_2 {0};
	hrt_abstime _next_pub_time_us{50};
};
