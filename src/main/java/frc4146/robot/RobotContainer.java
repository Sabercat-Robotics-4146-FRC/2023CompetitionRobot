package frc4146.robot;

import common.drivers.Gyroscope;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4146.robot.commands.autonomous.AlignRobotFiducial;
import frc4146.robot.commands.autonomous.BalanceRobot;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.gamepiece.ArmCommand;
import frc4146.robot.commands.gamepiece.ClawCommand;
import frc4146.robot.commands.gamepiece.PositionPiece;
import frc4146.robot.subsystems.*;

public class RobotContainer {

  private PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

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

    // re-centers robot orientation so that current heading = default heading
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));

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

  public Gyroscope getGyroscope() {
    return gyroscope;
  }
}
