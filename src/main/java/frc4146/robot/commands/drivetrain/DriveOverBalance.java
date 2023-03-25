package frc4146.robot.commands.drivetrain;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class DriveOverBalance extends CommandBase {
  public final DrivetrainSubsystem drivetrain;
  public final Pigeon pigeon;

  public final double s0speed = -0.4; // speed to hit the charge station at
  public final double s1speed = -0.225; // speed when driving over charge station
  public final double s2speed = -0.13; // speed when going down charge station
  public final double s3speed = -0.15;
  public final double s3distance = 0;

  public double s4time = 0;
  public SlewRateLimiter ff = new SlewRateLimiter(0.25);
  public SlewRateLimiter sf = new SlewRateLimiter(0.05);

  public int stage = 0;

  // Drives Backward Over Charge Station
  public DriveOverBalance(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
  }

  @Override
  public void initialize() {
    s4time = 0;
    stage = 0;
    pigeon.reset();
    drivetrain.resetPose(RigidTransform2.ZERO);
    ff.reset(s0speed / 3);
    sf.reset(s2speed);
  }

  @Override
  public void execute() {
    if (stage == 0) {
      drivetrain.drive(new Vector2(0, ff.calculate(s0speed)), 0, false);

      if (pigeon.getRoll() >= 5) {
        stage = 1;
        // drivetrain.drive(new Vector2(0, ff.calculate(0)), 0, false);
      }
    }
    if (stage == 1) {
      drivetrain.drive(new Vector2(0, ff.calculate(s1speed)), 0, false);

      if (pigeon.getRoll() <= -5) {
        stage = 2;
        // drivetrain.drive(new Vector2(0, ff.calculate(0)), 0, false);
      }
    }
    if (stage == 2) {

      drivetrain.drive(new Vector2(0, ff.calculate(s2speed)), 0, false);

      if (Math.abs(pigeon.getRoll()) <= 2) {
        stage = 4;
        drivetrain.resetPose(RigidTransform2.ZERO);
        // drivetrain.drive(new Vector2(0, ff.calculate(0)), 0, false);
      }
    }
    // else if (stage == 3) {
    //   drivetrain.drive(new Vector2(0, ff.calculate(s3speed)), 0, false);
    //   if (drivetrain.getPose().translation.length >= s3distance) {
    //     stage = 4;
    //   }
    // }
    if (stage == 4) {
      drivetrain.drive(new Vector2(0, sf.calculate(0)), 0, false);
    }
  }

  @Override
  public boolean isFinished() {
    return stage == 4 && drivetrain.getDriveSignal().getTranslation().length <= 0.02;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
