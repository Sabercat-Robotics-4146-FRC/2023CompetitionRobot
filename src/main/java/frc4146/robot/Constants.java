package frc4146.robot;

public class Constants {
  public static final int PRIMARY_CONTROLLER_PORT = 0;

  public static class DriveConstants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_STEER_ENCODER = 10;
    public static final double DRIVETRAIN_FRONT_LEFT_STEER_OFFSET =
        -Math.toRadians(15.380859374999998) - Math.toRadians(1.84);

    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 7;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_RIGHT_STEER_ENCODER = 12;
    public static final double DRIVETRAIN_FRONT_RIGHT_STEER_OFFSET =
        -Math.toRadians(245.21484375) - Math.toRadians(2.54);

    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_STEER_ENCODER = 9;
    public static final double DRIVETRAIN_BACK_LEFT_STEER_OFFSET =
        -Math.toRadians(189.66522216796875) + Math.toRadians(1.94);

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_RIGHT_STEER_ENCODER = 11;
    public static final double DRIVETRAIN_BACK_RIGHT_STEER_OFFSET =
        -Math.toRadians(239.14489746093753) - Math.toRadians(3.42 + 2.8);

    public static final int PIGEON_PORT = 16;

    public static final double TRACKWIDTH = 24.0;
    public static final double WHEELBASE = 24.0;
  }
}
