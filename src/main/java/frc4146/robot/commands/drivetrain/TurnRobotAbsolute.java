package frc4146.robot.commands.drivetrain;

import common.math.Vector2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.RobotContainer;
import frc4146.robot.subsystems.DrivetrainSubsystem;

public class TurnRobotCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;
  RobotContainer container;

  PIDController rotate;

  double speed = 0;
  double target = 0;

  public TurnRobotCommand(RobotContainer container, double target) {
    this.container = container;
    this.target = target;
  }

  @Override
  public void initialize() {
    double initDegree = container.getGyroscope().getAngle().toDegrees();

  }

  @Override
  public void execute() {
    container.getDrivetrainSubsystem().drive(Vector2.ZERO, 0.01);

    SmartDashboard.putNumber("Current Rotation", container.getGyroscope().getAngle().toDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    container.getDrivetrainSubsystem().drive(Vector2.ZERO, 0, false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(container.getGyroscope().getAngle().toDegrees() - target) < 10;
  }
}
