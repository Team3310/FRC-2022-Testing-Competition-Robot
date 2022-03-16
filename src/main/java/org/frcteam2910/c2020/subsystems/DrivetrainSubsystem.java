package org.frcteam2910.c2020.subsystems;

import java.util.Optional;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.Pigeon;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.DriveWithSetRotationCommand;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Limelight;
import org.frcteam2910.common.util.BallColor;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;

import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 0.34;
    public static final double WHEELBASE = 0.34;
    public static final double WHEEL_DIAMETER_INCHES = 3.64;  // Actual is 3.89"
    public TrapezoidProfile.Constraints constraints = new Constraints(6.0, 6.0);

    public ProfiledPIDController rotationController = new ProfiledPIDController(1.2, 0.03, 0.02, constraints, 0.02);
    public PIDController limelightController = new PIDController(1.7, 0.03, 0.02, 0.02);
    

    public enum DriveControlMode{
        JOYSTICKS, ROTATION, TRAJECTORY, LIMELIGHT, BALL_TRACKING
    }

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICKS;

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
            0.042746,
            0.0032181,
            0.30764
    );

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(), FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(12.5 * 12.0),
            new CentripetalAccelerationConstraint(15 * 12.0)
    };

    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.4, 0.0, 0.025),
            new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
    );

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
    );

    // private final SwerveDriveKinematics wpi_driveKinematics = new SwerveDriveKinematics(
    //         new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front left
    //         new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front right
    //         new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // back left
    //         new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    // );


    private final SwerveModule[] modules;

    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final Gyroscope gyroscope = new Pigeon(Constants.PIGEON_PORT);


    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private Vector2 velocity = Vector2.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    private XboxController primaryController;

    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        SwerveModule frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withPosition(0, 0)
                        .withSize(2, 3),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET
        );
        SwerveModule frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withPosition(2, 0)
                        .withSize(2, 3),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET
        );
        SwerveModule backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withPosition(4, 0)
                        .withSize(2, 3),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET
        );
        SwerveModule backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withPosition(6, 0)
                        .withSize(2, 3),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET
        );

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 3)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(1, 3)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(2, 3)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("Trajectory X", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().x;
                })
                .withPosition(0, 4)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
                    if (follower.getLastState() == null) {
                        return 0.0;
                    }
                    return follower.getLastState().getPathState().getPosition().y;
                })
                .withPosition(1, 4)
                .withSize(1, 1);

        tab.addNumber("Rotation Voltage", () -> {
                HolonomicDriveSignal signal;
                synchronized (stateLock) {
                    signal = driveSignal;
                }

                if (signal == null) {
                    return 0.0;
                }

                return signal.getRotation() * RobotController.getBatteryVoltage();
            })
            .withPosition(2, 4)
            .withSize(1, 1);

        tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity)
            .withPosition(3, 4)
            .withSize(1, 1);
    }

    public void setController(XboxController controller){
        primaryController = controller;
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

    public synchronized DriveControlMode getControlMode() {
        return driveControlMode;
    }

    public synchronized void setControlMode(DriveControlMode controlMode) {
        this.driveControlMode = controlMode;
    }


    public void joystickDrive() {    
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), getDriveRotationAxis().get(true), true);
    }

    public void setRotationTarget(double goal){
        rotationController.enableContinuousInput(0.0, Math.PI*2);
        rotationController.reset(getPose().rotation.toRadians());
        rotationController.setGoal(goal + getPose().rotation.toRadians());
        rotationController.setTolerance(0.087);
        setControlMode(DriveControlMode.ROTATION);
    }

    public boolean atRotationTarget(){
        
        if(rotationController.atGoal()){
            System.out.println("Reached target");
        }
        return rotationController.atGoal();
    }

    public void rotationDrive(){
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = rotationController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    // public void setLimelightTarget(){
    //     limelightController.enableContinuousInput(0.0, Math.PI*2);
    //     //limelightController.setSetpoint(goal + getPose().rotation.toRadians());
    //     //limelightController.setTolerance(0.087);
    //     setControlMode(DriveControlMode.LIMELIGHT);
    // }

    public void limelightDrive(){
        Limelight limelight = Limelight.getInstance();
        limelightController.setSetpoint(Math.toRadians(-limelight.getTargetHorizOffset()) + getPose().rotation.toRadians());

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = limelightController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    public void ballTrackDrive(){
        Limelight limelight = Limelight.getInstance();
        // Pipeline 0 on limelight is red (currently, at least)
        limelight.setPipeline(RobotContainer.getInstance().getSelectedBall() == BallColor.BLUE ? 1 : 0);
        limelightController.setSetpoint(Math.toRadians(-limelight.getTargetHorizOffset()) + getPose().rotation.toRadians());

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        double rotationOutput = limelightController.calculate(getPose().rotation.toRadians());

        drive(new Vector2(getDriveForwardAxis().get(true), getDriveStrafeAxis().get(true)), rotationOutput, true);
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }


    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getDriveVelocity());
        }
        return averageVelocity / 4;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * 39.37008 * WHEEL_DIAMETER_INCHES / 4.0);
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {

            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
        }
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        DriveControlMode i_controlMode = getControlMode();

        HolonomicDriveSignal currentDriveSignal = null;

        switch(i_controlMode){
            case JOYSTICKS:
                joystickDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case ROTATION:
                rotationDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case LIMELIGHT:
                limelightDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case BALL_TRACKING:
                ballTrackDrive();
                synchronized (stateLock) {
                    currentDriveSignal = this.driveSignal;
                }
                break;
            case TRAJECTORY:
                Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                        getPose(),
                        getVelocity(),
                        getAngularVelocity(),
                        time,
                        dt
                );
                if (trajectorySignal.isPresent()) {
                    currentDriveSignal = trajectorySignal.get();
                    currentDriveSignal = new HolonomicDriveSignal(
                        currentDriveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                        currentDriveSignal.getRotation() / RobotController.getBatteryVoltage(),
                        currentDriveSignal.isFieldOriented()
                    );
                }
                break; 
        }
       // System.out.println("output = " + currentDriveSignal.getRotation() + ", current = " + getPose().rotation.toRadians());
        updateModules(currentDriveSignal, dt);
        SmartDashboard.putNumber("Angle", getPose().rotation.toDegrees());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle rate", getAngularVelocity());
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());
    }
    

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }
}
