void setTalonSoftLimit(TalonFX& motor, int forwardLimit, int reverseLimit, int gearRatio)
{
    motor.ConfigForwardSoftLimitThreshold(forwardLimit/360*2048 * gearRatio, 0);
    motor.ConfigForwardSoftLimitEnable(true, 0);
    motor.ConfigForwardSoftLimitThreshold(reverseLimit/360*2048 * gearRatio, 0);
    motor.ConfigForwardSoftLimitEnable(true, 0);
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