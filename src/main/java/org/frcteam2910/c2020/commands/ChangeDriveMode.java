package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;

public class ChangeDriveMode extends CommandBase {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private DriveControlMode mode;

    public ChangeDriveMode(DrivetrainSubsystem.DriveControlMode mode) {
        addRequirements(drivetrain);
        this.mode = mode;
    }

    @Override
    public void initialize() {
        drivetrain.setControlMode(mode);
    }

    @Override
    public void execute() {}


    @Override
    public boolean isFinished() {return true;}

    @Override
    public void end(boolean interrupted) {}
}
