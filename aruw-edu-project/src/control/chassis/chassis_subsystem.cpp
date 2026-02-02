/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

namespace control::chassis
{


    
// STEP 1 (Tank Drive): create constructor
    ChassisSubsystem::ChassisSubsystem(Drivers& drivers, const ChassisConfig& config)
        : Subsystem()
        , desiredOutput{} //Receives target speed in m/s and converts to rpm for each motor within safety restrictions  (linear to rotational)
        , pidControllers{} //takes rpm and sends voltage
        , motors{} // communicates with motors to make wheel spin
        
// the desiredOutput, pidControllers and motors arrays must be constructed in the ChassisSubsystem’s constructor

// STEP 2 (Tank Drive): initialize function
   void ChassisSubsystem::initialize() { 
    for (auto &motor: motors)
    {
        motor.initialize();
    }
   }
// STEP 3 (Tank Drive): setVelocityTankDrive function
   void ChassisSubsystem::setVelocityTankDrive(float left, float right) { //convert to rpm, add safety, add to array
        // Convert linear speed to Raw RPM
        float leftRawRpm{mpsToRpm(left)};
        float rightRawRpm{mpsToRpm(right)};
        // Limit values to safe RPM range
        float leftRpm {limitVal(leftRawRpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM)};
        float rightRpm = {limitVal(rightRawRpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM)};
        // store value in array
        desiredOutput[static_cast<int>(MotorId::LF)] = leftRpm;
        desiredOutput[static_cast<int>(MotorId::LB)] = leftRpm;
        desiredOutput[static_cast<int>(MotorId::RF)] = rightRpm;        
        desiredOutput[static_cast<int>(MotorId::RB)] = rightRpm;
   }   
   
   
// STEP 4 (Tank Drive): refresh function
}  // namespace control::chassis
