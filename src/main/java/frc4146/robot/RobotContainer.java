package frc4146.robot;

import common.robot.DriverReadout;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.*;
import frc4146.robot.commands.subsystems.AlignWithFiducial;
=======
import frc4146.robot.commands.autonomous.AlignRobotFiducial;
import frc4146.robot.commands.autonomous.BalanceRobot;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.commands.subsystems.PositionPiece;
>>>>>>> Competition
import frc4146.robot.subsystems.*;

public class RobotContainer {

  public static DriverReadout driverInterface = new DriverReadout();

  private PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

  private final GenericHID secondaryRumble =
      new GenericHID(Constants.SECONDARY_CONTROLLER_PORT);

  private final Pigeon pigeon = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(pigeon);

  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  private final Trigger armState;

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

    armState = new Trigger(() -> arm.safeToDrive());

    CameraServer.startAutomaticCapture();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
<<<<<<< HEAD
    primaryController.getStartButton().onTrue(Commands.runOnce(pigeon::calibrate));
    primaryController
        .getStartButton()
        .onTrue(new InstantCommand(() -> drivetrainSubsystem.lockWheelsAngle(0)));
=======
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));

>>>>>>> Competition
    primaryController
        .getLeftBumperButton()
        .onTrue(Commands.runOnce(() -> drivetrainSubsystem.setMode(false)));
    primaryController
        .getRightBumperButton()
        .onTrue(Commands.runOnce(() -> drivetrainSubsystem.setMode(true)));

    primaryController.getAButton().onTrue(new BalanceRobot(drivetrainSubsystem, gyroscope));
    primaryController
        .getBButton()
        .toggleOnTrue(new AlignRobotFiducial(drivetrainSubsystem, limelight));
    primaryController
        .getXButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
        
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

<<<<<<< HEAD
    // primaryController.getBButton().onTrue(Commands.runOnce(drivetrainSubsystem::lockWheels));

    // primaryController.getBButton().toggleOnTrue(new BalanceRobot(drivetrainSubsystem,
    // gyroscope));

    primaryController.getBButton().onTrue(new AlignWithFiducial(drivetrainSubsystem, limelight));

    secondaryController.getAButton().onTrue(Commands.runOnce(arm::toggleExtensionMode));
    secondaryController.getBButton().onTrue(Commands.runOnce(arm::toggleRotationMode));

    // secondaryController.getBButton().toggleOnTrue(new ArmRotate(arm));

    armState.onFalse(
        new InstantCommand(
            () -> secondaryRumble.setRumble(GenericHID.RumbleType.kBothRumble, 0.5)));

    armState.onTrue(
        new InstantCommand(
            () -> secondaryRumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
=======
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
>>>>>>> Competition
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
