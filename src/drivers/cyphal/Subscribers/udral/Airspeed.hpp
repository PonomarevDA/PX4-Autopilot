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
 * @file Airspeed.hpp
 *
 * Defines basic functionality of Cyphal Airspeed subscription
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <uavcan/si/sample/velocity/Scalar_1_0.h>
#include <uavcan/si/sample/temperature/Scalar_1_0.h>
#include <uORB/topics/airspeed.h>

#include "../DynamicPortSubscriber.hpp"
#include <systemlib/mavlink_log.h>

class UavcanAirspeedSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanAirspeedSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "aspd.cas", instance)
	{
		_subj_sub.next = &_tas_sub;

		_tas_sub._subject_name = "aspd.tas";
		_tas_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_tas_sub._canard_sub.user_reference = this;
		_tas_sub.next = &_temperature_sub;

		_temperature_sub._subject_name = "aspd.temperature";
		_temperature_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_temperature_sub._canard_sub.user_reference = this;
		_temperature_sub.next = nullptr;

	};

	void subscribe() override
	{
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   uavcan_si_sample_velocity_Scalar_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _tas_sub._canard_sub.port_id,
					   uavcan_si_sample_velocity_Scalar_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_tas_sub._canard_sub);

		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _temperature_sub._canard_sub.port_id,
					   uavcan_si_sample_temperature_Scalar_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_temperature_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (receive.metadata.port_id == _subj_sub._canard_sub.port_id) {
			_report.timestamp = hrt_absolute_time();
			_report.indicated_airspeed_m_s = parseVelocity(receive);
			publishUorb();

		} else if (receive.metadata.port_id == _tas_sub._canard_sub.port_id) {
			_report.true_airspeed_m_s = parseVelocity(receive);

		} else if (receive.metadata.port_id == _temperature_sub._canard_sub.port_id) {
			_report.air_temperature_celsius = parseTemperature(receive) + 273.15f;
		}
	};

	float parseVelocity(const CanardRxTransfer &receive)
	{
		uavcan_si_sample_velocity_Scalar_1_0 msg {};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_si_sample_velocity_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bits);

		return msg.meter_per_second;
	}

	float parseTemperature(const CanardRxTransfer &receive)
	{
		uavcan_si_sample_temperature_Scalar_1_0 msg {};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_si_sample_temperature_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&msg_size_in_bits);

		return msg.kelvin;
	}

	void publishUorb()
	{
		if (_orb_advert == nullptr) {
			int instance = getInstance();
			_orb_advert = orb_advertise_multi(ORB_TOPIC, &_report, &instance);

		} else {
			(void)orb_publish(ORB_TOPIC, _orb_advert, &_report);
		}
	}

private:
	const orb_id_t ORB_TOPIC = ORB_ID(airspeed);
	orb_advert_t _orb_advert{nullptr};
	airspeed_s _report{};

	SubjectSubscription _tas_sub;
	SubjectSubscription _temperature_sub;
};
