package frc4146.robot;

public class Constants {
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

  public static class DriveConstants {

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR = 3;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_ENCODER = 12;
    public static final double DRIVETRAIN_FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(251.982);

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_MOTOR = 5;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_ENCODER = 14;
    public static final double DRIVETRAIN_FRONT_LEFT_STEER_OFFSET = -Math.toRadians(232.0313);

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_ENCODER = 16;
    public static final double DRIVETRAIN_BACK_LEFT_STEER_OFFSET = -Math.toRadians(329.766);

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_MOTOR = 9;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_ENCODER = 18;
    public static final double DRIVETRAIN_BACK_RIGHT_STEER_OFFSET = -Math.toRadians(335.48);

    public static final int PIGEON_PORT = 20;

    public static final double TRACKWIDTH = 20.0; // width
    public static final double WHEELBASE = 21.0; // front to rear wheels
  }

  public static class ArmConstants {
    public static final int ROTATION_LEFT_ID = 21;
    public static final int ROTATION_RIGHT_ID = 22;
    public static final int ROTATION_POT_CHANNEl = 0;

    public static final int EXTENSION_ID = 23;
    public static final int LOWER_LIMIT_CHANNEL = 9;
    public static final int UPPER_LIMIT_CHANNEL = 1;
    public static final int TICKS_PER_REVOLUTION = 2048;

    public static final double MAX_LENGTH = 63; // inches
    public static final double MIN_LENGTH = 39; // inches
    public static final double MAX_ANGLE = 1.05; // radians
    public static final double MIN_ANGLE = -1.57; // raidans

    public static final double POT_MAX_ROTATION = 0.35;
    public static final double POT_MIN_ROTATION = 0.623;

    public static final int CLAW_ID = 24;

  }

  public static class ClawConstants {
    public static final int CLAW_ID = 24;
    public static final int CLAW_POT_CHANNEL = 0;

    public static final double CLAW_CONE_MID = 796.0 / 1024.0;
    public static final double CLAW_CUBE = 0;
    public static final double CLAW_FULLY_OPENED = 0;
    public static final double CLAW_FULLY_CLOSED = 0;
  }

  public static class LimelightConstants {
    public static final double LIMELIGHT_ANGLE = 0;
    public static final double LIMELIGHT_HEIGHT = 49.25;
    public static final double LIMELIGHT_X_OFFSET = 7; // to the right is positive
    public static final double LIMELIGHT_Y_OFFSET = 4; // forward is positive

    public static final double DESIRED_FIDUCIAL_AREA = 1.29;

    public static final double D_Z = 1.0; // Alligned forward backward
    public static final double D_X = 0.0; // Alligned left right
  }
}
