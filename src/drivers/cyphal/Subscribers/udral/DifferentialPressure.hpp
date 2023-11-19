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
 * @file DifferentialPressure.hpp
 *
 * Defines basic functionality of Cyphal DifferentialPressure subscription
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <uavcan/si/sample/pressure/Scalar_1_0.h>
#include <lib/drivers/device/Device.hpp>
#include <uORB/topics/differential_pressure.h>

#include "../DynamicPortSubscriber.hpp"
#include <systemlib/mavlink_log.h>

class UavcanDiffPressureSubscriber : public UavcanDynamicPortSubscriber, public device::Device
{
public:
	UavcanDiffPressureSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "aspd.dpres", instance),
		Device("differential_pressure") {}

	void subscribe() override
	{
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   uavcan_si_sample_pressure_Scalar_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		uavcan_si_sample_pressure_Scalar_1_0 msg {};
		size_t payload_size = receive.payload_size;

		if (0 != uavcan_si_sample_pressure_Scalar_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&payload_size)) {
			return;
		}

		const hrt_abstime timestamp_sample = hrt_absolute_time();

		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_UAVCAN;
		_device_id.devid_s.address = getInstance();

		float diff_press_pa = msg.pascal;
		float temperature_c = 21.5f;

		differential_pressure_s report{};
		report.timestamp_sample = timestamp_sample;
		report.device_id = _device_id.devid;
		report.differential_pressure_pa = diff_press_pa;
		report.temperature = temperature_c + getInstance();
		report.timestamp = hrt_absolute_time();

		publishUorb(report);
	};

	void publishUorb(const differential_pressure_s& report)
	{
		if (_orb_advert == nullptr) {
			int instance = getInstance();
			_orb_advert = orb_advertise_multi(ORB_TOPIC, &report, &instance);

		} else {
			(void)orb_publish(ORB_TOPIC, _orb_advert, &report);
		}
	}

private:
	const orb_id_t ORB_TOPIC = ORB_ID(differential_pressure);
	orb_advert_t _orb_advert{nullptr};
};
