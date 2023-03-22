package frc4146.robot.commands.drivetrain;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class StraightLine extends CommandBase {
  public Pigeon pigeon;
  public DrivetrainSubsystem drivetrain;
  public double distance;

  public double min = 0.1;
  public double max = 0.25;

  public SlewRateLimiter ff = new SlewRateLimiter(0.7);

  public StraightLine(DrivetrainSubsystem drivetrain, Pigeon pigeon, double distance) {
    this.pigeon = pigeon;
    this.drivetrain = drivetrain;
    this.distance = distance;
    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.resetPose(RigidTransform2.ZERO);
    pigeon.reset();
    ff.calculate(0);
  }

  public void execute() {

    double dist_remaining = 1 - Math.abs(drivetrain.getPose().translation.length / distance);
    double trans_mag = Math.copySign(MathUtil.clamp(dist_remaining, min, max), -distance);
    double rot_mag = 0;
    drivetrain.drive(new Vector2(ff.calculate(trans_mag), 0), rot_mag);
  }

  public boolean isFinished() {
    double dist_traveled = Math.abs(drivetrain.getPose().translation.length);
    return (dist_traveled >= Math.abs(distance));
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
