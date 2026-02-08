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
    ChassisSubsystem::ChassisSubsystem(Drivers& drivers, const ChassisConfig& config) {
        desiredOutput{}; //Receives target speed in m/s and converts to rpm for each motor within safety restrictions  (linear to rotational)
        pidControllers{}; //takes rpm and sends voltage
        motors{}; // communicates with motors to make wheel spin
    }
// the desiredOutput, pidControllers and motors arrays must be constructed in the ChassisSubsystem’s constructor

// STEP 2 (Tank Drive): initialize function
   void ChassisSubsystem::initialize() { 
    for (auto &motor: motors) {
        motor.initialize();
    };
   }
// STEP 3 (Tank Drive): setVelocityTankDrive function {convert to rpm, add safety, add to array}
   void ChassisSubsystem::setVelocityTankDrive(float left, float right) { 
        // Convert linear speed to Raw RPM
        float leftRawRpm{mpsToRpm(left)};
        float rightRawRpm{mpsToRpm(right)};
        // Limit values to safe RPM range
        float leftDesiredRpm{limitVal(leftRawRpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM)};
        float rightDesiredRpm{limitVal(rightRawRpm, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM)};
        // store value in array
        desiredOutput[static_cast<int>(MotorId::LF)] = leftDesiredRpm;
        desiredOutput[static_cast<int>(MotorId::LB)] = leftDesiredRpm;
        desiredOutput[static_cast<int>(MotorId::RF)] = rightDesiredRpm;        
        desiredOutput[static_cast<int>(MotorId::RB)] = rightDesiredRpm;
   }   
   
   
// STEP 4 (Tank Drive): refresh function
   void ChassisSubsystem::refresh() {
    //call each pid controllers update function
    //input is desired RPM -actual RPM
        for (int i; (i < static_cast<int>(MotorId::NUM_MOTORS)); i++ ) {
        
        //assign variables to array value
        auto& pid{pidControllers[i]};
        auto& motor{motors[i]};
        
        // get motor actual rpm
        float motorActualRpm{motor.getShaftRPM()};
        // subtract desired from motor actual rpm
        float calculatedValue{(desiredOutput[static_cast<int>(MotorId::LF)] - motorActualRpm)};

        //update with value
        pid.update(calculatedValue - motorActualRpm);
        
        // call each motor setDesiredOutput function passing the pidControllers getValue function
        motor.setDesiredOutput(pid.getValue());

        };

   }

}  // namespace control::chassis
