package frc4146.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4146.robot.autonomous.AutonomousSelector;
import frc4146.robot.autonomous.AutonomousTab;
import frc4146.robot.commands.ArmCommand;
import frc4146.robot.commands.DriveCommand;
import frc4146.robot.subsystems.*;
import common.drivers.Gyroscope;
import common.robot.input.XboxController;

public class RobotContainer {

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);
      
  private final Gyroscope gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);
  private final Arm arm = new Arm();


  Command m_autoCommand;
  AutonomousSelector autoSelector;

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(arm);

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

    autoSelector = new AutonomousSelector(this);
    AutonomousTab tab = new AutonomousTab(this);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Configure Button Bindings
  }

  public AutonomousSelector getAutoSelector() {
    return autoSelector;
  }

  public Command getAutoCommand() {
    return autoSelector.getCommand();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
