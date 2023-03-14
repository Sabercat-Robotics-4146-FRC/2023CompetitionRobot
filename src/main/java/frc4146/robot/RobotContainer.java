package frc4146.robot;

import common.robot.DriverReadout;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.*;
import frc4146.robot.commands.subsystems.AlignWithFiducial;
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
    primaryController.getStartButton().onTrue(Commands.runOnce(pigeon::calibrate));
    primaryController
        .getStartButton()
        .onTrue(new InstantCommand(() -> drivetrainSubsystem.lockWheelsAngle(0)));
    primaryController
        .getYButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
    primaryController.getXButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleDriveFlag));

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
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
