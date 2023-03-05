void setTalonSoftLimit(TalonFX& motor, int forwardLimit, int reverseLimit, int gearRatio)
{
    motor.ConfigForwardSoftLimitThreshold(forwardLimit * 2048/360 * gearRatio, 0);
    motor.ConfigForwardSoftLimitEnable(true, 0);
    motor.ConfigReverseSoftLimitThreshold(reverseLimit * 2048/360 * gearRatio, 0);
    motor.ConfigReverseSoftLimitEnable(true, 0);
}

void configSwerveModule (TalonFX& drive, TalonFX& swerve, CANCoder& encoder, double encoderOffset)
{
    drive.SetNeutralMode(NeutralMode::Brake);
    drive.ConfigPeakOutputForward(mathConst::speedLimit);
    drive.ConfigPeakOutputReverse(-mathConst::speedLimit);
    
    swerve.SetNeutralMode(NeutralMode::Brake);
    encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    encoder.ConfigMagnetOffset(encoderOffset);
}

void limitSpeeds (TalonFX& motor, double speed)
{
    motor.ConfigPeakOutputForward(speed);
    motor.ConfigPeakOutputReverse(-speed);
}