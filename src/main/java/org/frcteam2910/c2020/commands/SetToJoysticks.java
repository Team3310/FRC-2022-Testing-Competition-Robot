package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;


public class SetToJoysticks extends CommandBase {
    private final DrivetrainSubsystem drivetrain;


    public SetToJoysticks(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setControlMode(DriveControlMode.JOYSTICKS);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {return true;}

    @Override
    public void end(boolean interrupted) {}
}
