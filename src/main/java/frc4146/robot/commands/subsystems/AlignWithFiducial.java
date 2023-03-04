package frc4146.robot.commands.subsystems;

import common.math.MathUtils;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.Constants.LimelightConstants;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Limelight;

public class AlignWithFiducial extends CommandBase {
  private final Limelight limelight;
  private final DrivetrainSubsystem drivetrain;

  double Kp_z = -0.05;
  double Kp_x = -0.05;

  double Kd_z = -0.05;
  double Kd_x = -0.05;

  double past_ex;
  double past_ez;

  double min_amt = 0.05;
  double max_amt = 0.2;

  public AlignWithFiducial(DrivetrainSubsystem drivetrain, Limelight limelight) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    double[] pos = limelight.getTargetPose();
    past_ex = pos[0] - LimelightConstants.D_X;
    past_ez = pos[2] - LimelightConstants.D_Z;
  }

  @Override
  public void execute() {
    double[] pos = limelight.getTargetPose();

    double ex = pos[0] - LimelightConstants.D_X;
    double ez = pos[2] - LimelightConstants.D_Z;

    double P_z = Kp_z * ez;
    double P_x = Kp_x * ex;

    double D_z = Kd_z * (ez - past_ez);
    double D_x = Kd_x * (ex - past_ex);

    double sp_z = P_z + D_z;
    double sp_x = P_x + D_x;

    past_ex = ex;
    past_ez = ez;

    sp_z = Math.copySign(MathUtil.clamp(Math.abs(sp_z), min_amt, max_amt), sp_z);

    SmartDashboard.putNumber("Sp_x", sp_x);
    SmartDashboard.putNumber("Sp_z", sp_z);

    SmartDashboard.putNumber("ex", ex);
    SmartDashboard.putNumber("ez", ez);
    drivetrain.drive(new Vector2(sp_z, -sp_x), 0);
  }

  public boolean isFinished() {
    double[] pos = limelight.getTargetPose();

    double ex = pos[0] - LimelightConstants.D_X;
    double ez = pos[2] - LimelightConstants.D_Z;

    return (Math.abs(ex) < 0.05 && Math.abs(ez) < 0.05);
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
