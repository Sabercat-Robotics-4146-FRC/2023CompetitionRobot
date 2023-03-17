package frc4146.robot.commands.fiducials;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.Constants.LimelightConstants;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Limelight;

public class DriveToFiducial extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Limelight limelight;

  public PIDController pid_fb;
  public PIDController pid_lr;

  double max_amt = 0.05;
  double min_amt = 0.004;

  public DriveToFiducial(DrivetrainSubsystem drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  public void initialize() {
    pid_fb = new PIDController(0.009, 0.001, 0.002);
    pid_fb.setTolerance(0.01, 0.01);
    pid_lr.setSetpoint(LimelightConstants.D_Z);
  }
}
