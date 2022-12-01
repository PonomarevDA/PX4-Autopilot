/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Rangefinder.hpp
 *
 * Defines basic functionality of Cyphal Rangefinder subscription
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <uavcan/si/sample/length/Scalar_1_0.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

#include "../DynamicPortSubscriber.hpp"

class UavcanRangefinderSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanRangefinderSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "range", instance) {}

	void subscribe() override
	{
		if (rangefinder == nullptr) {
			device::Device::DeviceId device_id{};
			device_id.devid_s.bus = 0;
			device_id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;
			device_id.devid_s.devtype = DRV_DIST_DEVTYPE_UAVCAN;
			device_id.devid_s.address = static_cast<uint8_t>(_instance);
			rangefinder = new PX4Rangefinder(device_id.devid, ROTATION_NONE);
		}


		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   uavcan_si_sample_length_Scalar_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (rangefinder == nullptr) {
			return;
		}

		uavcan_si_sample_length_Scalar_1_0 msg {};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_si_sample_length_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bits);

		uint8_t rangefinder_type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		rangefinder->set_rangefinder_type(rangefinder_type);
		rangefinder->set_fov(0.0);
		rangefinder->set_min_distance(0.2);  	// @todo
		rangefinder->set_max_distance(10.0);  	// @todo
		rangefinder->update(hrt_absolute_time(), msg.meter);
	};

private:
	PX4Rangefinder *rangefinder{nullptr};
	int _instance = 0;
};
