/****************************************************************************
 *
 * Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * @file FlightTaskContinuousYaw.cpp
 *
 */

#include "FlightTaskContinuousYaw.hpp"

bool FlightTaskContinuousYaw::activate(const vehicle_local_position_setpoint_s &last_setpoint){

    bool ret = FlightTask::activate(last_setpoint);

    _velocity_setpoint = _velocity;
    _yaw_setpoint = _yaw;
    _yawspeed_setpoint = 0.0f;
    // _velocity_setpoint(0) = -1.0f;
    // _origin_y = _position(0);
    _position_setpoint = {-16.0f, 0.0f , _position(2)};

    return ret;

}

bool FlightTaskContinuousYaw::update(){

    // float diff_y = _position(0) - _origin_y;

    // if (diff_y <= -16.0f){ //NED frame
    // _velocity_setpoint(0) = 1.0f;
    // }
    // else if (diff_y >= 0.0f){
    // _velocity_setpoint(0) = -1.0f;
    // }



    return true;
}
