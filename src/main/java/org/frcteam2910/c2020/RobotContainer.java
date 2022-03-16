package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.BasicDriveCommand;
import org.frcteam2910.c2020.commands.DriveWithSetRotationMP;
import org.frcteam2910.c2020.commands.SetToBallTrack;
import org.frcteam2910.c2020.commands.SetToJoysticks;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.BallColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    private final DriverReadout driverReadout;
    private static RobotContainer instance;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error building trajectories");
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrainSubsystem.setController(primaryController);

        //CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        
        //input from controller
        //CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        //input froom control and Limelight
        //CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveWithSetRotationCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), 30));

        driverReadout = new DriverReadout(this);

        configureButtonBindings();

        instance = this;
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );

        primaryController.getAButton().whenPressed(
                new BasicDriveCommand(drivetrainSubsystem, new Vector2(-0.5, 0.0), 0.0, false).withTimeout(0.3)
        );

        primaryController.getRightBumperButton().whenPressed(
                new SetToBallTrack(drivetrainSubsystem)
        );

        primaryController.getLeftBumperButton().whenPressed(
                new SetToJoysticks(drivetrainSubsystem)
        );

        SmartDashboard.putData("Change rotation to 90", new DriveWithSetRotationMP(drivetrainSubsystem, Math.PI/2));

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

    // private Axis getDriveForwardAxis() {
    //     return primaryController.getLeftYAxis();
    // }

    // private Axis getDriveStrafeAxis() {
    //     return primaryController.getLeftXAxis();
    // }

    // private Axis getDriveRotationAxis() {
    //     return primaryController.getRightXAxis();
    // }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public BallColor getSelectedBall() {
        return driverReadout.getSelectedBallColor();
    }

    public static RobotContainer getInstance() {
        return instance;
    }
}
