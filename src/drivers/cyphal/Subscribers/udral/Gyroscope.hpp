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
 * @file Gyroscope.hpp
 *
 * Defines basic functionality of Cyphal Gyroscope subscription
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include "uavcan/si/unit/angular_velocity/Vector3_1_0.h"
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#include "../DynamicPortSubscriber.hpp"

class UavcanGyroscopeSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanGyroscopeSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "gyro", instance) {}

	void subscribe() override
	{
		if (gyro == nullptr) {
			device::Device::DeviceId device_id{};
			device_id.devid_s.devtype = DRV_GYR_DEVTYPE_UAVCAN;
			device_id.devid_s.address = static_cast<uint8_t>(_instance);
			gyro = new PX4Gyroscope(device_id.devid, ROTATION_NONE);
		}

		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   uavcan_si_unit_angular_velocity_Vector3_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (gyro == nullptr) {
			return;
		}

		uavcan_si_unit_angular_velocity_Vector3_1_0 msg {};
		size_t msg_size_in_bytes = receive.payload_size;

		if (0 != uavcan_si_unit_angular_velocity_Vector3_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bytes)) {
			return;
		}

		const float x = msg.radian_per_second[0];
		const float y = msg.radian_per_second[1];
		const float z = msg.radian_per_second[2];

		gyro->update(hrt_absolute_time(), x, y, z);
	};

private:
	PX4Gyroscope *gyro{nullptr};
	int _instance = 0;
};
