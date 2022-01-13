package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.BasicDriveCommand;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error building trajectories");
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        driverReadout = new DriverReadout(this);
        driverReadout.getSelectedLoadingBay();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );

        primaryController.getAButton().whenPressed(
                new BasicDriveCommand(drivetrainSubsystem, new Vector2(-0.5, 0.0), 0.0, false).withTimeout(0.3)
        );

        // Manual hood adjustment
//        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(
//                () -> shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.HOOD_MAX_ANGLE) + HOOD_MANUAL_ADJUST_INTERVAL)
//        );
//        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
//                () -> shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.HOOD_MAX_ANGLE) - HOOD_MANUAL_ADJUST_INTERVAL)
//        );

        // Manual flywheel adjustment
//        primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
//                () -> shooterSubsystem.shootFlywheel(shooterSubsystem.getFlywheelTargetVelocity() + FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
//        primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
//                () -> shooterSubsystem.shootFlywheel(shooterSubsystem.getFlywheelTargetVelocity() - FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }
}
