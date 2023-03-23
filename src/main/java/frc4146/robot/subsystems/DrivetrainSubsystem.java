package frc4146.robot.subsystems;

import static frc4146.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import common.control.*;
import common.kinematics.ChassisVelocity;
import common.kinematics.SwerveKinematics;
import common.kinematics.SwerveOdometry;
import common.math.RigidTransform2;
import common.math.Rotation2;
import common.math.Vector2;
import common.robot.DriverReadout;
import common.robot.UpdateManager;
import common.util.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;

public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
  public boolean driveFlag = true;

  public DriverReadout _driverInterface = frc4146.robot.RobotContainer.driverInterface;

  // This value is used to turn the robot back to its initialPosition

  public ArrayList<Double> speeds;

  public Timer timer;

  public boolean fieldOriented = false;

  public boolean locked = false;

  /* The following objects are used to create accurate trajectory for the specific robot
   *
   * FEEDFORWARD_CONSTANTS informs TrajectoryConstraints, HolonomicTrajectoryFollower
   *                       contains data on velocity, acceleration
   *
   * TRAJECTORY_CONSTRAINTS identifies a profile for the generated trajectories
   *                        contains FEEDFORWARD_CONSTANTS, other acceleration limits
   */
  public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS =
      new DrivetrainFeedforwardConstants(0.8198, 0.27975, -0.28894);

  public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
    new FeedforwardConstraint(
        12.0, // TODO: test 12
        FEEDFORWARD_CONSTANTS.getVelocityConstant(),
        FEEDFORWARD_CONSTANTS.getAccelerationConstant(),
        true), // TODO: was false, want to test true
    new MaxAccelerationConstraint(12.5 * 12.0),
    new CentripetalAccelerationConstraint(5.0)
  };

  /** follower uses PID, feedforward control to create trajectories */
  private final HolonomicMotionProfiledTrajectoryFollower follower =
      new HolonomicMotionProfiledTrajectoryFollower(
          new PidConstants(2.0, 0.01, 0.001),
          new PidConstants(2.0, 0.01, 0.001),
          new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

  /* swerveKinematics contains a set of vectors,
   *  each one corresponding to one swerve module,
   *  in the direction of that wheel's projected motion
   */
  private final SwerveKinematics swerveKinematics =
      new SwerveKinematics(
          new Vector2(
              DriveConstants.TRACKWIDTH / 2.0, DriveConstants.WHEELBASE / 2.0), // front left
          new Vector2(
              DriveConstants.TRACKWIDTH / 2.0, -DriveConstants.WHEELBASE / 2.0), // front right
          new Vector2(
              -DriveConstants.TRACKWIDTH / 2.0, DriveConstants.WHEELBASE / 2.0), // back left
          new Vector2(
              -DriveConstants.TRACKWIDTH / 2.0, -DriveConstants.WHEELBASE / 2.0) // back right
          );

  private final SwerveModule[] modules;
  private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private final TalonFX[] talons;

  private final Pigeon gyroscope;

  /** swerveOdometry tracks the robot's position over time, using encoder data */
  private final SwerveOdometry swerveOdometry =
      new SwerveOdometry(
          swerveKinematics,
          RigidTransform2.ZERO); // starting at (0, 0) with initial angle 0 degrees

  private RigidTransform2 pose = RigidTransform2.ZERO; // (x, y, heading)
  private Vector2 velocity = Vector2.ZERO;
  private double angularVelocity = 0.0;

  private double last_pigeon_angle = 0.0;

  /** driveSignal holds "live" data on pose, which determines how robot drives */
  private HolonomicDriveSignal driveSignal;

  /** odometry entries are robot data, logged to NetworkTable */
  private final GenericEntry odometryXEntry; // robot's x position

  private final GenericEntry odometryYEntry; // robot's y position
  private final GenericEntry odometryAngleEntry; // robot's heading/angle

  private boolean brake_mode = false;

  public Rotation2 desired_heading;

  public PIDController drift_correction = new PIDController(0.005, 0, 0);

  public DrivetrainSubsystem(Pigeon gyro) {

    drift_correction.enableContinuousInput(-180, 180);

    gyroscope = gyro;

    // gyroscope.setInverted(false);
    driveSignal = new HolonomicDriveSignal(new Vector2(0, 0), 0.0, true);

    timer = new Timer();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    frontLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_STEER_OFFSET);
    frontRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(4, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_STEER_OFFSET);
    backLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(6, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_BACK_LEFT_STEER_OFFSET);
    backRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(8, 0)
                .withSize(2, 4),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_ENCODER,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_STEER_OFFSET);

    modules =
        new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
    TalonFX leftb = new TalonFX(DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
    leftb.setInverted(true);
    TalonFX leftf = new TalonFX(DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
    leftf.setInverted(true);
    talons =
        new TalonFX[] {
          leftf,
          new TalonFX(DriveConstants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
          leftb,
          new TalonFX(DriveConstants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)
        };

    for (var talon : talons) {
      talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.5));
      talon.configOpenloopRamp(0); // # seconds to reach peak throttle
    }

    // sets up Shuffleboard to receive odometry data
    odometryXEntry = tab.add("X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    odometryYEntry = tab.add("Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    odometryAngleEntry = tab.add("Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();

    tab.addNumber(
            "Trajectory X",
            () -> {
              if (follower.getLastState() == null) {
                return 0.0;
              }
              return follower.getLastState().getPathState().getPosition().x;
            })
        .withPosition(1, 0)
        .withSize(1, 1);
    tab.addNumber(
            "Trajectory Y",
            () -> {
              if (follower.getLastState() == null) {
                return 0.0;
              }
              return follower.getLastState().getPathState().getPosition().y;
            })
        .withPosition(1, 1)
        .withSize(1, 1);

    tab.addNumber(
        "Rotation Voltage",
        () -> {
          HolonomicDriveSignal signal;
          signal = driveSignal;
          if (signal == null) {
            return 0.0;
          }

          return signal.getRotation() * RobotController.getBatteryVoltage();
        });

    tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);
    tab.addBoolean("Drive Enabled", () -> driveFlag);
    tab.addBoolean("Field Oriented", () -> fieldOriented);

    tab.addNumber("Roll", () -> gyroscope.getRoll());
    tab.addNumber("Pitch", () -> gyroscope.getPitch());
    tab.addNumber("Yaw", () -> gyroscope.getRate());

    _driverInterface
        .primaryLayout
        .addBoolean("Field Oriented", () -> fieldOriented)
        .withPosition(0, 3);
    _driverInterface.primaryLayout.addBoolean("Drive Enabled", () -> driveFlag).withPosition(0, 2);
    _driverInterface.primaryLayout.add("Drive Heading", gyroscope).withPosition(0, 0);
  }

  /** updates driveSignal with desired translational, rotational velocities */
  public void drive(
      Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
    double tx = translationalVelocity.x;
    double ty = translationalVelocity.y;

    if (Math.abs(tx) < 0.005) {
      tx = 0;
    }
    if (Math.abs(ty) < 0.005) {
      ty = 0;
    }
    double mag = Math.hypot(tx, ty);
    double rotDeadband = 0.002;
    if (mag <= 0.005) rotDeadband = 0.003;
    if (Math.abs(rotationalVelocity) < rotDeadband) {
      rotationalVelocity = 0;
    }

    if (rotationalVelocity == 0 && Math.abs(tx) + Math.abs(ty) > 0.1) {
      double adjustment_mag =
          MathUtil.clamp(
              drift_correction.calculate(gyroscope.getAngle() % 360, last_pigeon_angle),
              -0.05,
              0.05);
      adjustment_mag = Math.abs(adjustment_mag) < 0.0003 ? 0 : adjustment_mag;
      rotationalVelocity = adjustment_mag;
    }
    last_pigeon_angle = gyroscope.getAngle() % 360;
    driveSignal =
        new HolonomicDriveSignal(new Vector2(tx, ty), rotationalVelocity, isFieldOriented);
  }

  public void drive(Vector2 translationalVelocity, double rotationalVelocity) {
    drive(translationalVelocity, rotationalVelocity, fieldOriented);
  }

  /** updates odometry data, to be posted and read by drive functions */
  private void updateOdometry(double time, double dt) {
    Vector2[] moduleVelocities = getModuleVelocities();
    Rotation2 angle = Rotation2.fromDegrees(gyroscope.getAngle());
    double angularVelocity = gyroscope.getRate();

    ChassisVelocity velocity =
        swerveKinematics.toChassisVelocity(
            moduleVelocities); // translates individual module velocities into composite velocity
    // for robot

    this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
    this.velocity = velocity.getTranslationalVelocity();
    this.angularVelocity = angularVelocity;
  }

  /** sets module values, as read from drive signal */
  private void updateModules(HolonomicDriveSignal driveSignal, double dt) {

    if (driveFlag) {
      Rotation2 rotOffset =
          (driveSignal.isFieldOriented())
              ? Rotation2.fromDegrees(gyroscope.getAngle())
              : Rotation2.ZERO;

      ChassisVelocity chassisVelocity =
          new ChassisVelocity(
              driveSignal.getTranslation().rotateBy(rotOffset), driveSignal.getRotation());

      Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
      SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
      for (int i = 0; i < moduleOutputs.length; i++) {
        if (locked) modules[i].set(-moduleOutputs[i].length * 12.0, 0);
        else
          modules[i].set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
      }
    }
  }

  @Override
  public void update(double time, double dt) {
    updateOdometry(time, dt);
    HolonomicDriveSignal driveSignal;
    Optional<HolonomicDriveSignal> trajectorySignal =
        follower.update(getPose(), getVelocity(), getAngularVelocity(), time, dt);
    driveSignal = trajectorySignal.orElseGet(() -> this.driveSignal);
    if (driveFlag) updateModules(driveSignal, dt);
  }

  @Override
  public void periodic() {
    updateDisplay();
  }

  /** updates NetworkTable with data read from robot pose */
  public void updateDisplay() {
    RigidTransform2 pose = getPose();
    odometryXEntry.setDouble(pose.translation.x);
    odometryYEntry.setDouble(pose.translation.y);
    odometryAngleEntry.setDouble(pose.rotation.toDegrees());
  }

  public void toggleLocked(boolean l) {
    locked = l;
  }

  public void toggleLocked() {
    locked = !locked;
  }

  public void lockWheelsAngle(double angle) {
    if (getAverageAbsoluteValueVelocity() < 5.0) {
      frontLeftModule.set(0, angle * 2 * Math.PI / 180);
      frontRightModule.set(0, angle * 2 * Math.PI / 180);
      backLeftModule.set(0, angle * 2 * Math.PI / 180);
      backRightModule.set(0, angle * 2 * Math.PI / 180);
    }
  }

  public void setMode(boolean brake) {
    brake_mode = brake;
    for (TalonFX talon : talons) {
      if (brake) talon.setNeutralMode(NeutralMode.Brake);
      else talon.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void toggleMode() {
    setMode(!brake_mode);
  }

  public double getAverageAbsoluteValueVelocity() {
    return Arrays.stream(modules).mapToDouble(m -> Math.abs(m.getDriveVelocity())).sum()
        / modules.length;
  }

  public Vector2[] getModuleVelocities() {
    return Arrays.stream(modules)
        .map(
            m ->
                Vector2.fromAngle(Rotation2.fromRadians(m.getSteerAngle()))
                    .scale(m.getDriveVelocity() * 39.37008)) // inches in meter
        .toArray(Vector2[]::new);
  }

  // public void drift_correct(ChassisVelocity speeds) {
  //   Vector2 trans = speeds.getTranslationalVelocity();
  //   double xy = Math.abs(trans.x) + Math.abs(trans.y);
  //   double ang_velocity = speeds.getAngularVelocity();
  //   if (speeds.getAngularVelocity() <= 0.0 || pXY <= 0) {
  //     desired_heading = getPose().rotation;
  //   } else if (xy > 0) {
  //     ang_velocity +=
  //         drift_correction.calculate(getPose().rotation.toDegrees(),
  // desired_heading.toDegrees());
  //   }
  //   pXY = xy;
  // }

  public RigidTransform2 getPose() {
    return pose;
  }

  public HolonomicDriveSignal getDriveSignal() {
    return driveSignal;
  }

  public HolonomicMotionProfiledTrajectoryFollower getFollower() {
    return follower;
  }

  public Vector2 getVelocity() {
    return velocity;
  }

  public double getAngularVelocity() {
    return angularVelocity;
  }

  public void toggleFieldOriented() {
    fieldOriented = !fieldOriented;
  }

  public void toggleDriveFlag() {
    driveFlag = !driveFlag;
  }

  public void resetPose(RigidTransform2 pose) {
    this.pose = pose;
    swerveOdometry.resetPose(pose);
  }

  public void resetGyroAngle(Rotation2 angle) {
    gyroscope.reset();
  }
}
