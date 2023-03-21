package frc4146.robot.commands.fiducials;

import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Limelight;

public class AlignWithTarget extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Limelight limelight;

  public PIDController pid_rot;

  public PIDController pid_fb;
  public PIDController pid_lr;

  double max_amt = 0.05;
  double min_amt = 0.004;

  double count = 0;

  public AlignWithTarget(DrivetrainSubsystem drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  public void initialize() {
    pid_rot = new PIDController(0.009, 0.001, 0.002);
    pid_rot.setTolerance(1.0 / 27.0, 0.2 / 27.0);
    pid_rot.setSetpoint(0);

    pid_lr = new PIDController(0.01, 0.001, 0.002);
    pid_lr.setTolerance(0.01, 0.02);
    pid_lr.setSetpoint(0);

    pid_fb = new PIDController(0.01, 0.001, 0.002);
    pid_fb.setTolerance(0.01, 0.02);
    pid_fb.setSetpoint(0);
  }

  public void execute() {
    if (limelight.getSeesTarget()) {
      double r_mag = pid_rot.calculate(getError()[0]);
      drivetrain.drive(
          Vector2.ZERO,
          Math.copySign(MathUtil.clamp(Math.abs(r_mag), min_amt, max_amt), r_mag),
          true);
      if (Math.abs(getError()[0]) <= 0.05) count += 1;

    } else {
      drivetrain.drive(Vector2.ZERO, 0.03);
    }
  }

  public boolean isFinished() {
    return limelight.getSeesTarget() && count >= 10 && pid_rot.atSetpoint();
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0.0);
  }

  public double[] getError() {
    return new double[] {-(limelight.getHorizontalOffset()) / 27};
  }
}
