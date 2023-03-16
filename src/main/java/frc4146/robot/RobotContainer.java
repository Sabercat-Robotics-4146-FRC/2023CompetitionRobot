package frc4146.robot;

import common.robot.DriverReadout;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.AlignWithFiducial;
import frc4146.robot.commands.autonomous.AlignRobotFiducial;
import frc4146.robot.commands.autonomous.BalanceRobot;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.commands.subsystems.PositionPiece;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.drivetrain.TestDriveCommand;
import frc4146.robot.commands.gamepiece.ArmCommand;
import frc4146.robot.commands.gamepiece.ClawCommand;
import frc4146.robot.commands.gamepiece.PositionPiece;

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

  private final Pigeon gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);

  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  private final Trigger armState;

  public boolean testMode;

  public RobotContainer() {

    pdh.setSwitchableChannel(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    // CommandScheduler.getInstance().registerSubsystem(arm);
    // CommandScheduler.getInstance().registerSubsystem(claw);

    // note: test mode must be configured in this tab before enabling, can also change default value in code
    testMode =
        Shuffleboard.getTab("Test Mode")
            .add("Test Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .getEntry()
            .getBoolean(false);

    if (testMode) {
      CommandScheduler.getInstance()
          .setDefaultCommand(
              drivetrainSubsystem,
              new TestDriveCommand(
                  drivetrainSubsystem,
                  primaryController.getLeftYAxis(),
                  primaryController.getLeftXAxis(),
                  primaryController.getRightXAxis()));
    } else {
      CommandScheduler.getInstance()
          .setDefaultCommand(
              drivetrainSubsystem,
              new DriveCommand(
                  drivetrainSubsystem,
                  primaryController.getLeftYAxis(),
                  primaryController.getLeftXAxis(),
                  primaryController.getRightXAxis()));
    }

    CommandScheduler.getInstance()
        .setDefaultCommand(
            arm,
            new ArmCommand(
                arm,
                secondaryController.getLeftTriggerAxis(),
                secondaryController.getRightTriggerAxis(),
                secondaryController.getRightYAxis(),
                testMode));

    CommandScheduler.getInstance()
        .setDefaultCommand(
            claw, new ClawCommand(claw, secondaryController.getLeftXAxis(), testMode));

    armState = new Trigger(() -> arm.safeToDrive());

    CameraServer.startAutomaticCapture();

    configureButtonBindings();
  }

  private void configureButtonBindings() {


    // re-centers robot orientation so that current heading = default heading
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));
    
    primaryController
        .getStartButton()
        .onTrue(new InstantCommand(() -> drivetrainSubsystem.lockWheelsAngle(0)));

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

    // while holding LEFT bumper, press other button to go to setpoint for CONE
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

    armState.onFalse(
        new InstantCommand(
            () -> secondaryRumble.setRumble(GenericHID.RumbleType.kBothRumble, 0.5)));

    armState.onTrue(
        new InstantCommand(
            () -> secondaryRumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

    // while holding RIGHT bumper, press other button to go to setpoint for CUBE
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

  public Claw getClawSubsystem() {
    return claw;
  }

  public Gyroscope getGyroscope() {
    return gyroscope;
  }

}
