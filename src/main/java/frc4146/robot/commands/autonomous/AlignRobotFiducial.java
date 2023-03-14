package frc4146.robot.commands.autonomous;

import common.math.MathUtils;
import common.math.Vector2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.Constants.LimelightConstants;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Limelight;

public class AlignRobotFiducial extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Limelight limelight;

  public int stage = 0;

  public PIDController pid_rot;
  public PIDController pid_lr;
  public PIDController pid_fb;

  public AlignRobotFiducial(DrivetrainSubsystem drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;

    Shuffleboard.getTab("Drivetrain").addNumber("Stage", () -> stage);

    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.setMode(true);

    pid_fb = new PIDController(-0.17, 0.0, -0.01);
    pid_fb.setTolerance(0.01, 0.01);

    pid_lr = new PIDController(0.17, 0.0, -0.01);
    pid_lr.setTolerance(0.01, 0.01);

    pid_rot = new PIDController(0.6 * 0.0125, 1.2 * 0.0125, 3 * 0.0125);
    pid_rot.setTolerance(0.05, 0.01);

    pid_rot.setSetpoint(0);
    pid_fb.setSetpoint(0);
    pid_lr.setSetpoint(0);
  }

  public void execute() {
    if (stage == 0) {
      drivetrain.drive(
          Vector2.ZERO, MathUtils.clamp(pid_rot.calculate(getRotationError()), -0.01, 0.01));
      if (pid_rot.atSetpoint()) {
        stage += 1;
      }
    }
    if (stage == 1) {
      drivetrain.drive(
          new Vector2(0, MathUtils.clamp(pid_lr.calculate(getLeftRightError()), -0.1, 0.1)),
          0,
          false);
      if (pid_lr.atSetpoint() && pid_rot.atSetpoint()) {
        stage += 1;
      }
    }
    if (stage == 2) {
      drivetrain.drive(
          new Vector2(MathUtils.clamp(pid_fb.calculate(getForwardBackError()), -0.1, 0.1), 0),
          0,
          false);
    }
  }

  public boolean isFinished() {
    return pid_lr.atSetpoint() && pid_rot.atSetpoint() && pid_fb.atSetpoint();
  }

  public double getForwardBackError() {
    double[] pos = limelight.getTargetPose();
    return pos[2] - LimelightConstants.D_Z;
  }

  public double getLeftRightError() {
    double[] pos = limelight.getTargetPose();
    return pos[0] - LimelightConstants.D_X;
  }

  public double getRotationError() {
    return limelight.getHorizontalOffset() / 27;
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
    pid_lr.reset();
    pid_fb.reset();
    pid_rot.reset();
  }
}
