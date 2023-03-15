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
#endif
