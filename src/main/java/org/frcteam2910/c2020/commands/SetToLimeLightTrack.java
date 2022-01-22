package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;

public class SetToLimeLightTrack extends CommandBase {
    private final DrivetrainSubsystem drivetrain;


    public SetToLimeLightTrack(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setControlMode(DriveControlMode.LIMELIGHT);
    }

    @Override
    public void execute() {}


    @Override
    public boolean isFinished() {return true;}

    @Override
    public void end(boolean interrupted) {}
}
