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
 * @file Gripper.hpp
 *
 * Defines basic functionality of Cyphal Gripper publisher
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#pragma once

#include <uavcan/primitive/scalar/Integer8_1_0.h>
#include "../Publisher.hpp"

class GripperPublisher : public UavcanPublisher
{
public:
	GripperPublisher(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(handle, pmgr, "udral.", "gripper", instance)
	{

	};

	~GripperPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		if (!_vehicle_command_sub.updated() || _port_id == CANARD_PORT_ID_UNSET || _port_id == 0) {
			return;
		}

		vehicle_command_s vehicle_command;
		_vehicle_command_sub.update(&vehicle_command);

		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_DO_GRIPPER) {
			return;
		}


		uavcan_primitive_scalar_Integer8_1_0 msg;
		msg.value = vehicle_command.param2;

		uint8_t buffer[uavcan_primitive_scalar_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
		size_t payload_size = uavcan_primitive_scalar_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
		uavcan_primitive_scalar_Integer8_1_0_serialize_(&msg, buffer, &payload_size);

		const CanardTransferMetadata transfer_metadata = {
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = _port_id,
			.remote_node_id = CANARD_NODE_ID_UNSET,
			.transfer_id    = _transfer_id,
		};

		_transfer_id++;
		_canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				      &transfer_metadata,
				      payload_size,
				      &buffer);
	};

private:
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
};
