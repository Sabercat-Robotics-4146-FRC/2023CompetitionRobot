package frc4146.robot.commands.drivetrain;

import common.math.MathUtils;
import common.math.Vector2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.RobotContainer;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class TurnRobotAbsolute extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Pigeon gyroscope;

  PIDController rotate;

  double target = 0;
  double initDegree = 0;
  double direction = 1;

  public TurnRobotAbsolute(Pigeon gyroscope, DrivetrainSubsystem drivetrain, double target) {
    this.gyroscope = gyroscope;
    this.drivetrain = drivetrain;
    this.target = target;
  }

  @Override
  public void initialize() {
    gyroscope.reset();
    initDegree = gyroscope.getAngle() % 360;
    direction = Math.abs(target - initDegree) > 180 ? -1 : 1;
  }

  @Override
  public void execute() {
    double mag = MathUtils.clamp((gyroscope.getAngle() - initDegree) / Math.abs(target - initDegree), 0.01, 0.05);
    drivetrain.drive(Vector2.ZERO, mag * direction);

    SmartDashboard.putNumber("Current Rotation", gyroscope.getAngle());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0, false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(gyroscope.getAngle() - target) < 10;
  }
}

