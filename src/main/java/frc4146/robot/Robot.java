package frc4146.robot;

import common.math.RigidTransform2;
import common.robot.UpdateManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private static Robot instance = null;
  private RobotContainer robotContainer = new RobotContainer();
  private UpdateManager updateManager = new UpdateManager(robotContainer.getDrivetrainSubsystem());

  private Command autonomousCommand = null;

  public Robot() {

    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    updateManager.startLoop(5.0e-3);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void teleopInit() {

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // Brake mode on wasn't a problem, so we may be able to keep it.
    robotContainer.getDrivetrainSubsystem().setMode(false);
  }
}
