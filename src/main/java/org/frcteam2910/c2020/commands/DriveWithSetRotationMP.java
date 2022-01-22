package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class DriveWithSetRotationMP extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private double rotationTarget;


    public DriveWithSetRotationMP(DrivetrainSubsystem drivetrain, double rotationTarget) {
        this.drivetrain = drivetrain;
        this.rotationTarget = rotationTarget;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        drivetrain.setRotationTarget(rotationTarget);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished(){
        return drivetrain.atRotationTarget();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControlMode(DriveControlMode.JOYSTICKS);
    }
}
