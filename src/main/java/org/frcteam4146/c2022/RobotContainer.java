package org.frcteam4146.c2022;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam4146.c2022.commands.commandGroups.ShootBallCommand;
import org.frcteam4146.c2022.commands.drive.AimRobotCommand;
import org.frcteam4146.c2022.commands.drive.DriveCommand;
import org.frcteam4146.c2022.commands.subsystems.ToggleLimelightTrackingCommand;
import org.frcteam4146.c2022.subsystems.*;
import org.frcteam4146.common.robot.input.XboxController;

public class RobotContainer {

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Flywheel flywheel = new Flywheel();
  private final Indexer indexer = new Indexer();
  private final Servos servos = new Servos();
  private final Limelight limelight = new Limelight(servos);

  public RobotContainer() {

    // primaryController.getLeftXAxis().setInverted(true);
    // primaryController.getRightXAxis().setInverted(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(flywheel);
    CommandScheduler.getInstance().registerSubsystem(indexer);
    CommandScheduler.getInstance().registerSubsystem(servos);
    CommandScheduler.getInstance().registerSubsystem(limelight);

    CommandScheduler.getInstance()
        .setDefaultCommand(
            drivetrainSubsystem,
            new DriveCommand(
                drivetrainSubsystem,
                primaryController.getLeftXAxis(),
                primaryController.getLeftYAxis(),
                primaryController.getRightXAxis()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Configure Button Bindings
    primaryController.getBButton().whenPressed(new ShootBallCommand(limelight, flywheel, indexer));
    primaryController.getAButton().whenPressed(new ToggleLimelightTrackingCommand(limelight));
    primaryController.getXButton().whenPressed(new AimRobotCommand(drivetrainSubsystem, limelight));

  }

  // public Command getAutonomousCommand() {
  //   return autonomousChooser.getCommand(this);
  // }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
  public Flywheel getFlywheelSubsystem()  {return flywheel;}
  public Indexer getIndexerSubsystem() {return indexer;}
  public Servos getServosSubsystem() {return servos;}
  public Limelight getLimelightSubsystem() {return limelight;}

}
