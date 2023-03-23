package frc4146.robot.commands.drivetrain;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class StraightLine2 extends CommandBase {
  public Pigeon pigeon;
  public DrivetrainSubsystem drivetrain;
  public double distance;
  public double angle;

  public double min = 0.25;
  public double max = 0.45;

  public SlewRateLimiter ff = new SlewRateLimiter(0.7);

  public StraightLine2(
      DrivetrainSubsystem drivetrain, Pigeon pigeon, double angle, double distance) {
    this.pigeon = pigeon;
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.angle = angle;
    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.resetPose(RigidTransform2.ZERO);
    pigeon.reset();
    ff.calculate(0);
  }

  public void execute() {

    double dist_remaining = 1 - Math.abs(drivetrain.getPose().translation.length / distance);
    double trans_mag = ff.calculate(MathUtil.clamp(dist_remaining, min, max));

    drivetrain.drive(
        new Vector2(
            trans_mag * Math.cos((180 + angle) * Math.PI / 180),
            trans_mag * Math.sin((180 + angle) * Math.PI / 180)),
        0,
        false);
  }

  public boolean isFinished() {
    double dist_traveled = Math.abs(drivetrain.getPose().translation.length);
    return (dist_traveled >= Math.abs(distance));
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
