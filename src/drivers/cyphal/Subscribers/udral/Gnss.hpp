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
 * @file Gnss.hpp
 *
 * Defines basic functionality of Cyphal GNSS subscription
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

// UDRAL Specification Messages
#include <reg/udral/physics/kinematics/geodetic/PointStateVarTs_0_1.h>
#include <uavcan/primitive/scalar/Integer16_1_0.h>
#include <uORB/topics/sensor_gps.h>

#include "../DynamicPortSubscriber.hpp"

class UavcanGnssSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanGnssSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "gps", instance)
	{
		_subj_sub.next = &_sats_sub;

		_sats_sub._subject_name = "gps.sats";
		_sats_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_sats_sub._canard_sub.user_reference = this;
		_sats_sub.next = &_status_sub;

		_status_sub._subject_name = "gps.status";
		_status_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_status_sub._canard_sub.user_reference = this;
		_status_sub.next = &_pdop_sub;

		_pdop_sub._subject_name = "gps.pdop";
		_pdop_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_pdop_sub._canard_sub.user_reference = this;
		_pdop_sub.next = nullptr;
	};

	void subscribe() override
	{
		if (_subj_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _subj_sub._canard_sub.port_id,
						   reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_subj_sub._canard_sub);
		}

		if (_sats_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _sats_sub._canard_sub.port_id,
						   uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_sats_sub._canard_sub);
		}

		if (_status_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _status_sub._canard_sub.port_id,
						   uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_status_sub._canard_sub);
		}

		if (_pdop_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _pdop_sub._canard_sub.port_id,
						   uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_pdop_sub._canard_sub);
		}
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (receive.metadata.port_id == _subj_sub._canard_sub.port_id) {
			parsePoint(receive);
			publishUorb();

		} else if (receive.metadata.port_id == _sats_sub._canard_sub.port_id) {
			parseSats(receive);

		} else if (receive.metadata.port_id == _status_sub._canard_sub.port_id) {
			parseStatus(receive);

		} else if (receive.metadata.port_id == _pdop_sub._canard_sub.port_id) {
			parsePdop(receive);
		}
	};

	void parsePoint(const CanardRxTransfer &receive)
	{
		reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1 geo {};
		size_t geo_size_in_bytes = receive.payload_size;

		if (0 != reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_deserialize_(&geo,
				(const uint8_t *)receive.payload,
				&geo_size_in_bytes)) {
			return;
		}

		_report.timestamp = hrt_absolute_time();
		_report.latitude_deg = M_RAD_TO_DEG * geo.value.position.value.latitude;
		_report.longitude_deg = M_RAD_TO_DEG * geo.value.position.value.longitude;
		_report.altitude_msl_m = geo.value.position.value.altitude.meter;
		_report.altitude_ellipsoid_m = _report.altitude_msl_m;

		_report.vel_n_m_s = geo.value.velocity.value.meter_per_second[0];
		_report.vel_e_m_s = geo.value.velocity.value.meter_per_second[1];
		_report.vel_d_m_s = geo.value.velocity.value.meter_per_second[2];
		_report.vel_m_s = sqrtf(_report.vel_n_m_s * _report.vel_n_m_s +
					_report.vel_e_m_s * _report.vel_e_m_s +
					_report.vel_d_m_s * _report.vel_d_m_s);
		_report.cog_rad = atan2f(_report.vel_e_m_s, _report.vel_n_m_s);
		_report.vel_ned_valid = true;
	}

	void parseSats(const CanardRxTransfer &receive)
	{
		_report.satellites_used = parseInteger16(receive);
	}

	void parseStatus(const CanardRxTransfer &receive)
	{
		_report.fix_type = parseInteger16(receive);
	}

	void parsePdop(const CanardRxTransfer &receive)
	{
		_report.hdop = parseInteger16(receive);
		_report.vdop = _report.hdop;
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
	int16_t parseInteger16(const CanardRxTransfer &receive)
	{
		uavcan_primitive_scalar_Integer16_1_0 msg;
		size_t size_in_bytes = receive.payload_size;

		if (0 != uavcan_primitive_scalar_Integer16_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&size_in_bytes)) {
			return 0;
		}

		return msg.value;
	}

	const orb_id_t ORB_TOPIC = ORB_ID(sensor_gps);
	orb_advert_t _orb_advert{nullptr};

	int _instance = 0;
	sensor_gps_s _report{};

	SubjectSubscription _sats_sub;
	SubjectSubscription _status_sub;
	SubjectSubscription _pdop_sub;
};
