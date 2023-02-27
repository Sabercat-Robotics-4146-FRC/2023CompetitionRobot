package frcteam4146.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frcteam4146.robot.autonomous.AutonomousSelector;
import frcteam4146.robot.autonomous.AutonomousTab;
import frcteam4146.robot.commands.DriveCommand;
import frcteam4146.robot.subsystems.*;

import common.robot.input.XboxController;

public class RobotContainer {

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  Command m_autoCommand;
  AutonomousSelector autoSelector;

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

    CommandScheduler.getInstance()
        .setDefaultCommand(
            drivetrainSubsystem,
            new DriveCommand(
                drivetrainSubsystem,
                primaryController.getLeftYAxis(),
                primaryController.getLeftXAxis(),
                primaryController.getRightXAxis()));

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
