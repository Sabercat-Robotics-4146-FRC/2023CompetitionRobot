package frc4146.robot.commands.drivetrain;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class StraightLine extends CommandBase {
  public Pigeon pigeon;
  public DrivetrainSubsystem drivetrain;
  public double distance;

  public boolean faster;

  public double min;
  public double max;

  public StraightLine(
      DrivetrainSubsystem drivetrain, Pigeon pigeon, double distance, boolean faster) {
    this.pigeon = pigeon;
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.faster = faster;
    addRequirements(drivetrain);
  }

  public void initialize() {
    min = faster ? 0.125 : 0.1;
    max = faster ? 0.375 * 1.15 : 0.375;

    drivetrain.resetPose(RigidTransform2.ZERO);
    pigeon.reset();
  }

  public void execute() {

    double dist_remaining = 1 - Math.abs(drivetrain.getPose().translation.length / distance);
    drivetrain.drive(
        new Vector2(Math.copySign(MathUtil.clamp(dist_remaining, min, max), -distance), 0), 0);
  }

  public boolean isFinished() {
    double dist_traveled = Math.abs(drivetrain.getPose().translation.length);
    return (dist_traveled >= Math.abs(distance));
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
