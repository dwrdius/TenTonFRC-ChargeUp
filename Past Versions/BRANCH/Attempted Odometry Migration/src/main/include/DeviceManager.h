#ifndef DEVICES_H
#define DEVICES_H

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <AHRS.h>

#include "constants.h"

extern TalonFX m_FLDriveMotor;
extern TalonFX m_FRDriveMotor;
extern TalonFX m_BLDriveMotor;
extern TalonFX m_BRDriveMotor;

extern TalonFX m_FLSwerveMotor;
extern TalonFX m_FRSwerveMotor;
extern TalonFX m_BLSwerveMotor;
extern TalonFX m_BRSwerveMotor;

extern CANCoder FLCANCoder;
extern CANCoder FRCANCoder;
extern CANCoder BLCANCoder;
extern CANCoder BRCANCoder;

extern AHRS navX;

extern frc::XboxController m_Controller;

extern void initializeAllComponents();

#endif