package frc4146.robot;

import common.drivers.Gyroscope;
import common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.subsystems.*;

public class RobotContainer {

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private final Gyroscope gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    // enables drive using controller
    CommandScheduler.getInstance()
        .setDefaultCommand(
            drivetrainSubsystem,
            new DriveCommand(
                drivetrainSubsystem,
                primaryController.getLeftYAxis(),
                primaryController.getLeftXAxis(),
                primaryController.getRightXAxis()));

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    // TODO: Configure Button Bindings
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));

    primaryController.getYButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
    primaryController.getXButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleDriveFlag));

    }
  public DrivetrainSubsystem getDrivetrainSubsystem() {return drivetrainSubsystem;}
  public Gyroscope getGyroscope() {return gyroscope;}

}
