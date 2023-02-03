#ifndef MATHSTUFF_H
#define MATHSTUFF_H
#include <cmath>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace mathConst {
    const double rotationVectorMultiplier = 2; // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
    const double speedLimit = 0.2;             // limit motor speed output   0 < x <= 1
    const double driveExponent = 3;

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
    const double xCoords[4] = {-7.25,7.25,-7.25,7.25};
    const double yCoords[4] = {11.625,11.625,-11.625,-11.625};
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
        constexpr double kFrontLeft{-31.289063};
        constexpr double kFrontRight{171.914063};
        constexpr double kBackLeft{56.162109};
        constexpr double kBackRight{-116.982422};
}
#endif