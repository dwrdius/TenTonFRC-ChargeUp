#ifndef MATHSTUFF_H
#define MATHSTUFF_H

#include <cmath>

#include <frc/geometry/Translation2d.h>
#include <frc/util/Color.h>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace CanIDs {
    // Drive Motors
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

    // CANCAN CANivore    
    //Intake Talon
    const int kIntakeUpDown = 1;

    //Arm talons
    const int kArmMotor = 2;
    const int kShooterBottom = 3;
    const int kShooterTop = 4;

}

namespace RevIDs {
// TEMP IDs -------------------------------------------------------------------------------------------------------------------------

    // Intake Motors Neo
    const int kIntakeLeader = 14;
    const int kIntakeFollower = 13;

    // LED
    const int kLED = 1;    
}

namespace mathConst {
    const double speedLimit = 0.7;             // limit motor speed output   0 < x <= 1
    double pseudoSpeedLimit = speedLimit;
    // rotation:translation ratio
    double rotationVectorMultiplier = 0.4; 

    // const double baseWidth = 15.5;             // inches
    // const double baseLength = 25.0;

    // speed of swivel
    const double swerveMotorSpeed = 0.4
    ;

    // relative coordinate of centre of wheels to the centre of the robot [FL, FR, BL, FR]
    double xCoords[4] = {-7.75, 7.75, -7.75, 7.75};
    double yCoords[4] = {12.5, 12.5, -12.5, -12.5};
    
    // 360 / circumference of turn = degrees/inch; verified by simulation
    const double kDegreesPerInchDenominator = 360.0 / (M_PI*2*sqrt(pow(mathConst::xCoords[0], 2) + pow(mathConst::yCoords[0], 2)));

    const double intakeGearRatio = 30*32/16;
    const double armGearRatio = 30;
}

namespace Deadbands {
    const double joyDead = 0.2; // DO NOT DECREASE (controller drift was insane)
    const double deadOffset = 1/(1-joyDead);

    const double swerveDeadband = 0.005;
    const double AngleDead = 2; // degrees    
    
    const double LLDistanceDead = 5; // inches
}

namespace Slews {
    const double driveSlew = 0.05;
    const double LLSlew = 0.02;
}

namespace Limelight {
    // how many degrees back is your limelight rotated from perfectly vertical?
    const double limelightMountAngleDegrees = -20;

    // distance from the center of the Limelight lens to the floor
    const double limelightLensHeightInches = 38.875;

    // distance from the target to the floor
    const double midHeightInches = 23.5;
    const double tallHeightInches = 43.75;    

    const int TopReflective = 1;
    const int BottomReflective = 2;
    const int April = 3;
    const int Cone = 4;
}
    
namespace Colours {
    static const frc::Color KYellowTarget = frc::Color(0.4, 0.45, 0.1);
    static const frc::Color KPurpleTarget = frc::Color(0.27, 0.4, 0.2);
} 

namespace Alliances {
    int Red = 1;
    int Blue = -1;
}

namespace CANCoderOffsets {
        constexpr double kFrontLeft{55.810547};
        constexpr double kFrontRight{84.111328};
        constexpr double kBackLeft{56.074219};
        constexpr double kBackRight{-116.015625};
}

namespace Odometry {
    const frc::Translation2d kFLLocation{12.5_in, 7.75_in};
    const frc::Translation2d kFRLocation{12.5_in, -7.75_in};
    const frc::Translation2d kBLLocation{-12.5_in, 7.75_in};
    const frc::Translation2d kBRLocation{-12.5_in, -7.75_in};
}

namespace Exponentiation {
    const double nexus = 0.587;
    
    const double arb_a = 0.497914;
    const double arb_b = 21.4711;
    const double arb_c = 0.374127;
    const double arb_d = -0.000182195;
    const double arb_e = 0.0904269;
    const double arb_f = 8.87463;
    const double arb_g = 0.800117;
    const double arb_h = 0.46705;
}









#endif