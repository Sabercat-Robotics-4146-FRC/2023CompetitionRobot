package frc4146.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4146.robot.Constants.LimelightConstants;

public class Limelight implements Subsystem {
  public static NetworkTable mLime;

  public Limelight() {
    mLime = NetworkTableInstance.getDefault().getTable("limelight");

    setPipelineType(0);
  }

  public double getDistanceToTarget(double targetHeight) {
    double angle = getVerticalOffset() + LimelightConstants.LIMELIGHT_ANGLE;
    return (targetHeight - LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(angle));
  }

  public boolean getSeesTarget() {
    return mLime.getEntry("tv").getBoolean(true);
  }

  public double getHorizontalOffset() {
    return mLime.getEntry("tx").getDouble(0.0);
  }

  public double getVerticalOffset() {
    return mLime.getEntry("ty").getDouble(0.0);
  }

  public double getTargetArea() {
    return mLime.getEntry("ta").getDouble(0.0);
  }

  public double getAprilTagID() {
    return mLime.getEntry("tid").getDouble(0.0);
  }

  public double[] getTargetPose() {
    // X, Y, Z, Roll, Pitch, Yaw. The important ones are the first 3
    setPipelineType(0);
    return mLime.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public double[] getRobotPose() {
    // X, Y, Z, Roll, Pitch, Yaw. The important ones are the first 3
    setPipelineType(0);
    return mLime.getEntry("robotpose_targetspace").getDoubleArray(new double[6]);
  }

  public void setPipelineType(int t) {
    // 0 - fiducial detection, 1 - retroreflective tape detection

    mLime.getEntry("pipeline").setNumber(t);
  }
}
