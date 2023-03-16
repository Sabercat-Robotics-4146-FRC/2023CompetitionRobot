package frc4146.robot;

import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4146.robot.commands.autonomous.AlignWithTarget;
import frc4146.robot.commands.autonomous.BalanceRobot;
import frc4146.robot.commands.autonomous.SimpleTrajectory;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.commands.subsystems.PositionPiece;
import frc4146.robot.commands.subsystems.ScorePiece;
import frc4146.robot.subsystems.*;

public class RobotContainer {

  private PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  // private final DriveJoysticks joysticks = new DriveJoysticks(0, 1);

  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

  private final Pigeon gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);

  private final Limelight limelight = new Limelight();

  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

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

    primaryController.getAButton().onTrue(new BalanceRobot(drivetrainSubsystem, gyroscope));

    primaryController
        .getXButton()
        .toggleOnTrue(new SimpleTrajectory(drivetrainSubsystem, gyroscope, arm, claw));

    // primaryController
    //     .getXButton()
    //     .toggleOnTrue(new StraightLine(drivetrainSubsystem, gyroscope, -200));
    // // primaryController
    // //     .getXButton()
    // //     .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));

    primaryController
        .getYButton()
        .toggleOnTrue(new AlignWithTarget(drivetrainSubsystem, limelight));

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
  }

  public Arm getArmSubsystem() {
    return arm;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public Pigeon getGyroscope() {
    return gyroscope;
  }

  public Command getAutonomousCommand() {
    return new SimpleTrajectory(drivetrainSubsystem, gyroscope, arm, claw);
  }
}
