package frc4146.robot.commands.fiducials;

import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Limelight;

public class AlignWithTarget extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Limelight limelight;

  public PIDController pid_rot;

  public PIDController pid_fb;
  public PIDController pid_lr;

  public SlewRateLimiter ff = new SlewRateLimiter(100);
  public SlewRateLimiter sf = new SlewRateLimiter(100);
  public SlewRateLimiter rf = new SlewRateLimiter(.2);

  double max_amt = 0.07;
  double min_amt = 0.005;

  double count = 0;

  public AlignWithTarget(DrivetrainSubsystem drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  public void initialize() {

    pid_rot = new PIDController(0.003, 0.000, 0.0);
    pid_rot.setTolerance(2.0 / 27.0);
    pid_rot.setSetpoint(0);

    pid_lr = new PIDController(-0.7, 0, 0.00);
    pid_lr.setTolerance(0.05);
    pid_lr.setSetpoint(0);

    pid_fb = new PIDController(0.7, 0.000, 0.00);
    pid_fb.setTolerance(0.05);

    drivetrain.resetGyroAngle(null);
  }

  public void execute() {
    if (limelight.getSeesTarget()) {
      double[] error = getError();
      double r_mag = rf.calculate(pid_rot.calculate(error[0]));
      double fb_mag = pid_fb.calculate(error[1], 1.2);
      double lr_mag = pid_lr.calculate(error[2]);
      if (pid_fb.atSetpoint()) fb_mag = 0;
      if (pid_lr.atSetpoint()) lr_mag = 0;

      drivetrain.drive(
          new Vector2(
              Math.copySign(MathUtil.clamp(Math.abs(fb_mag), 0.01, 0.2), fb_mag),
              Math.copySign(MathUtil.clamp(Math.abs(lr_mag), 0.01, 0.1), lr_mag)),
          0,
          // Math.copySign(MathUtil.clamp(Math.abs(r_mag), 0.000, 0.02), r_mag),
          false);
      if (Math.abs(getError()[0]) <= 0.05) count += 1;
    } else {
      drivetrain.drive(Vector2.ZERO, 0);
    }

    // } else {
    //   drivetrain.drive(Vector2.ZERO, 0.03, true);
    // }
  }

  public boolean isFinished() {
    return limelight.getSeesTarget()
        && pid_fb.atSetpoint()
        // && pid_rot.atSetpoint()
        && pid_lr.atSetpoint();
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0.0, true);
  }

  public double[] getError() {
    double[] tP = limelight.getTargetPose();
    return new double[] {-(limelight.getHorizontalOffset()) / 27, tP[2], tP[0]};
  }
}
