package frc4146.robot;

import common.robot.DriverReadout;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4146.robot.commands.autonomous.AlignWithTarget;
import frc4146.robot.commands.autonomous.BalanceRobot;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.commands.subsystems.PositionPiece;
import frc4146.robot.commands.subsystems.ScorePiece;
import frc4146.robot.subsystems.*;
import frc4146.robot.util.AutonomousSelector;
import frc4146.robot.util.AutonomousTab;

public class RobotContainer {

  public static DriverReadout driverInterface = new DriverReadout();

  private PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);
  private final GenericHID secondaryRumble = new GenericHID(Constants.SECONDARY_CONTROLLER_PORT);

  private final Pigeon gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);
  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  private final AutonomousSelector autonomousSelector;
  private final AutonomousTab autonomousTab;

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(arm);
    CommandScheduler.getInstance().registerSubsystem(claw);

    CommandScheduler.getInstance()
        .setDefaultCommand(
            drivetrainSubsystem,
            new DriveCommand(
                drivetrainSubsystem,
                primaryController.getLeftYAxis(),
                primaryController.getLeftXAxis(),
                primaryController.getRightXAxis()));

    CommandScheduler.getInstance()
        .setDefaultCommand(
            arm,
            new ArmCommand(
                arm,
                secondaryController.getLeftTriggerAxis(),
                secondaryController.getRightTriggerAxis(),
                secondaryController.getRightYAxis()));
    CommandScheduler.getInstance()
        .setDefaultCommand(claw, new ClawCommand(claw, secondaryController.getLeftXAxis()));

    CameraServer.startAutomaticCapture();

    autonomousSelector = new AutonomousSelector(this);
    autonomousTab = new AutonomousTab(this);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::reset));

    primaryController
        .getLeftBumperButton()
        .onTrue(Commands.runOnce(() -> drivetrainSubsystem.setMode(false)));
    primaryController
        .getRightBumperButton()
        .onTrue(Commands.runOnce(() -> drivetrainSubsystem.setMode(true)));

    primaryController.getAButton().toggleOnTrue(new BalanceRobot(drivetrainSubsystem, gyroscope));
    primaryController.getBButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleLocked));
    primaryController
        .getXButton()
        .toggleOnTrue(new AlignWithTarget(drivetrainSubsystem, limelight));
    primaryController
        .getYButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));

    // primaryController
    //     .getXButton()
    //     .toggleOnTrue(new SimpleTrajectory(drivetrainSubsystem, gyroscope, arm, claw));

    // primaryController
    //     .getXButton()
    //     .toggleOnTrue(new StraightLine(drivetrainSubsystem, gyroscope, -200));
    // primaryController
    //     .getXButton()
    //     .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));

    // primaryController
    //     .getYButton()
    //     .toggleOnTrue(new AlignWithTarget(drivetrainSubsystem, limelight));

    // primaryController
    //     .getBButton()
    //     .onTrue(new InstantCommand(() -> drivetrainSubsystem.lockWheelsAngle(0)));

    secondaryController.getStartButton().toggleOnTrue(new ScorePiece(arm, claw, "cone", "high"));

    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getXButton())
        .toggleOnTrue(new PositionPiece(arm, "cone", "intake"));
    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getAButton())
        .toggleOnTrue(new PositionPiece(arm, "cone", "low"));
    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getBButton())
        .toggleOnTrue(new PositionPiece(arm, "cone", "mid"));
    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getYButton())
        .toggleOnTrue(new PositionPiece(arm, "cone", "high"));

    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getXButton())
        .toggleOnTrue(new PositionPiece(arm, "cube", "intake"));
    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getAButton())
        .toggleOnTrue(new PositionPiece(arm, "cube", "low"));
    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getBButton())
        .toggleOnTrue(new PositionPiece(arm, "cube", "mid"));
    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getYButton())
        .toggleOnTrue(new PositionPiece(arm, "cube", "high"));

    // armState
    //     .and(() -> drivetrainSubsystem.getAverageAbsoluteValueVelocity() > 0.5)
    //     .onFalse(
    //         new InstantCommand(
    //             () -> secondaryRumble.setRumble(GenericHID.RumbleType.kBothRumble, 0.5)));

  }

  public Arm getArmSubsystem() {
    return arm;
  }

  public Claw getClawSubsystem() {
    return claw;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public Pigeon getGyroscope() {
    return gyroscope;
  }

  public AutonomousSelector getAutonomousSelector() {
    return autonomousSelector;
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.getCommand();
  }
}
