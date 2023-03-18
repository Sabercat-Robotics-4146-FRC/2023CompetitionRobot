package frc4146.robot.commands.drivetrain;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;

public class StraightLineCommand extends CommandBase {

  public DrivetrainSubsystem drivetrain;
  public double distance;
  public double angle;

  public double min = 0.1;
  public double max = 0.375;

  public StraightLineCommand(DrivetrainSubsystem drivetrain, double distance, double angle) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.angle = angle;
    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.resetPose(RigidTransform2.ZERO);
  }

  public void execute() {

    double dist_remaining = 1 - Math.abs(drivetrain.getPose().translation.length / distance);
    double trans_mag = Math.copySign(MathUtil.clamp(dist_remaining, min, max), -distance);
    double rot_mag = 0;
    drivetrain.drive(
        new Vector2(trans_mag * Math.cos(angle), trans_mag * Math.sin(angle)), rot_mag, false);
  }

  public boolean isFinished() {
    double dist_traveled = Math.abs(drivetrain.getPose().translation.length);
    return (dist_traveled >= Math.abs(distance));
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
