#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/numbers>

namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace drivetrainConstants {
    //CAN IDs
    constexpr int kMotorDriveFL = 2;
    constexpr int kMotorDriveFR = 1;
    constexpr int kMotorDriveBL = 3;
    constexpr int kMotorDriveBR = 4;

    constexpr int kMotorTurnFL = 6;
    constexpr int kMotorTurnFR = 5;
    constexpr int kMotorTurnBL = 7;
    constexpr int kMotorTurnBR = 8;

    constexpr int kEncoderTurnFL = 10;
    constexpr int kEncoderTurnFR = 9;
    constexpr int kEncoderTurnBL = 11;
    constexpr int kEncoderTurnBR = 12;

    constexpr double kEncoderOffsetFL{0};
    constexpr double kEncoderOffsetFR{0};
    constexpr double kEncoderOffsetBL{0};
    constexpr double kEncoderOffsetBR{0};

    namespace calculations {
        constexpr auto kFinalDriveRatio{6.75 * 360_deg};
        constexpr units::length::inch_t kWheelCircumference = {2 * wpi::numbers::pi * 3.8_in / 2};

        constexpr auto kModuleMaxSpeed{16.3_fps};
        constexpr auto kChassisMaxSpeed{16.3_fps};

        constexpr auto kModuleMaxAngularVelocity{wpi::numbers::pi * 1_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{wpi::numbers::pi * 2_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}