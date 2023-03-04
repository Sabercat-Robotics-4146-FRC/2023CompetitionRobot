package frc4146.robot;

import common.drivers.Gyroscope;
import common.robot.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4146.robot.commands.drivetrain.*;
import frc4146.robot.commands.subsystems.*;
import frc4146.robot.subsystems.*;

public class RobotContainer {

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

  private final Gyroscope gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  public RobotContainer() {
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

    // enables drive using controller

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));
    primaryController.getStartButton().onTrue(Commands.runOnce(drivetrainSubsystem::zeroWheels));
    primaryController
        .getYButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
    primaryController.getXButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleDriveFlag));

    secondaryController
        .getBButton()
        .toggleOnTrue(new InstantCommand(() -> arm.extendArm(arm.getPos())));
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public Gyroscope getGyroscope() {
    return gyroscope;
  }
}
