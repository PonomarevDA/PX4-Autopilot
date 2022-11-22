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
 * @file Barometer.hpp
 *
 * Defines basic functionality of Cyphal Barometer subscription
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <uavcan/si/sample/pressure/Scalar_1_0.h>
#include <uavcan/si/sample/temperature/Scalar_1_0.h>
#include <uORB/topics/sensor_baro.h>
#include <lib/geo/geo.h>
#include <lib/drivers/device/Device.hpp>

#include "../DynamicPortSubscriber.hpp"

class UavcanBarometerSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanBarometerSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "baro.pressure", instance)
	{
		_subj_sub.next = &_temperature_sub;

		_temperature_sub._subject_name = "baro.temperature";
		_temperature_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_temperature_sub._canard_sub.user_reference = this;
		_temperature_sub.next = nullptr;

		device::Device::DeviceId device_id{};
		device_id.devid_s.bus = 0;
		device_id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;

		device_id.devid_s.devtype = DRV_BARO_DEVTYPE_UAVCAN;
		device_id.devid_s.address = static_cast<uint8_t>(_instance);

		_report.device_id = device_id.devid;
		_report.error_count = 0;
	};

	void subscribe() override
	{
		if (_subj_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _subj_sub._canard_sub.port_id,
						   uavcan_si_sample_pressure_Scalar_1_0_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_subj_sub._canard_sub);
		}

		if (_temperature_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _temperature_sub._canard_sub.port_id,
						   uavcan_si_sample_temperature_Scalar_1_0_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_temperature_sub._canard_sub);
		}
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (receive.metadata.port_id == _subj_sub._canard_sub.port_id) {
			parsePressure(receive);
			publishUorb();

		} else if (receive.metadata.port_id == _temperature_sub._canard_sub.port_id) {
			parseTemperature(receive);
		}
	};

	void parsePressure(const CanardRxTransfer &receive)
	{
		uavcan_si_sample_pressure_Scalar_1_0 msg {};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_si_sample_pressure_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bits);

		_report.timestamp_sample = hrt_absolute_time();
		_report.pressure = msg.pascal;
		_report.timestamp = hrt_absolute_time();
	}

	void parseTemperature(const CanardRxTransfer &receive)
	{
		uavcan_si_sample_temperature_Scalar_1_0 msg {};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_si_sample_temperature_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bits);

		if (PX4_ISFINITE(msg.kelvin) && (msg.kelvin >= 0.f)) {
			_report.temperature = msg.kelvin - 273.15f;

		} else {
			_report.temperature = NAN;
		}
	}

	void publishUorb()
	{
		if (_orb_advert == nullptr) {
			_orb_advert = orb_advertise_multi(ORB_TOPIC, &_report, &_instance);

		} else {
			(void)orb_publish(ORB_TOPIC, _orb_advert, &_report);
		}
	}

private:
	const orb_id_t ORB_TOPIC = ORB_ID(sensor_baro);
	orb_advert_t _orb_advert{nullptr};

	int _instance = 0;
	sensor_baro_s _report{};

	SubjectSubscription _temperature_sub;
};
