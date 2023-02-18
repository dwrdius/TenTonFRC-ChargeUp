#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cmath>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace mathConst {
    const double rotationVectorMultiplier = 2; // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
    const double speedLimit = 0.2;             // limit motor speed output   0 < x <= 1
    const double driveExponent = 2;

    const double baseWidth = 14.5;             // inches
    const double baseLength = 23.25;

    // speed of swivel
    const double swerveMotorSpeed = 0.5;

    const double slew = 0.1;

    const double swerveDeadband = 0.005;
    const double deadband = 0.2; // DO NOT DECREASE (controller drift was insane)
    const double deadbandOffset = 1/(1-deadband);

    // For determining motor positions on an x,y grid
    const double relativeX = baseWidth / 2;
    const double relativeY = baseLength / 2;

    // relative coordinate of centre of wheels to the centre of the robot [FL, FR, BL, FR]
    static double xCoords[4] = {-7.75, 7.75, -7.75, 7.75};
    static double yCoords[4] = {12.5, 12.5, -12.5, -12.5};
}

namespace CanIDs {
    //CAN IDs
    const int kFLDriveMotor = 3;
    const int kFRDriveMotor = 1;
    const int kBLDriveMotor = 2;
    const int kBRDriveMotor = 4;

    // Swerve Motors
    const int kFLSwerveMotor = 7;
    const int kFRSwerveMotor = 5;
    const int kBLSwerveMotor = 6;
    const int kBRSwerveMotor = 8;

    // Encoders
    const int kFLCANCoder = 11;
    const int kFRCANCoder = 9;
    const int kBLCANCoder = 10;
    const int kBRCANCoder = 12;
}
namespace CANCoderOffsets {
        constexpr double kFrontLeft{-124.189453};
        constexpr double kFrontRight{-95.888672};
        constexpr double kBackLeft{-123.925781};
        constexpr double kBackRight{63.984375};
}

namespace kinematicsValues
{
    const frc::Translation2d kFrontLeftLocationFromCenter{12.5_in, 7.75_in};
    const frc::Translation2d kFrontRightLocationFromCenter{12.5_in, -7.75_in};
    const frc::Translation2d kBackLeftLocationFromCenter{-12.5_in, 7.75_in};
    const frc::Translation2d kBackRightLocationFromCenter{-12.5_in, -7.75_in};    
}

#endif