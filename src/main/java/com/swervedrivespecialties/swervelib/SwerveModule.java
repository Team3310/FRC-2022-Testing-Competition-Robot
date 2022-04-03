package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void resetAbsoluteSteerAngle();

    void setEncoderAutoResetIterations(int iterations);    

    void setMotorNeutralMode(NeutralMode modeType);

    void set(double driveVoltage, double steerAngle);
}
