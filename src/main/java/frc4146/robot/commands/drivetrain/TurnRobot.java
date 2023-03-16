package frc4146.robot.commands.drivetrain;

import common.math.Rotation2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class TurnRobot extends CommandBase {
  public final DrivetrainSubsystem drivetrainSubsystem;
  public final Pigeon gyroscope;
  public Rotation2 degrees;

  public Rotation2 currDegrees;
  public double driveTurn = 0;
  public double old_driveturn = 0;

  public double initChange;

  public double multiplier = 1;

  public boolean sweep = true; // = true;

  public TurnRobot(DrivetrainSubsystem drivetrainSubsystem, Pigeon gyroscope, double degrees) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.initChange = degrees;
    this.gyroscope = gyroscope;

    addRequirements(drivetrainSubsystem);
  }

  public TurnRobot(
      DrivetrainSubsystem drivetrainSubsystem, Pigeon gyroscope, double degrees, boolean sweep) {
    this(drivetrainSubsystem, gyroscope, degrees);
    this.sweep = true;
  }

  @Override
  public void initialize() {
    degrees =
        Rotation2.fromDegrees(initChange).rotateBy(Rotation2.fromDegrees(gyroscope.getAngle()));
    multiplier = 1;
  }

  @Override
  public void execute() {
    this.currDegrees = Rotation2.fromDegrees(gyroscope.getAngle());
    driveTurn =
        -Rotation2.fromDegrees(0).rotateBy(degrees).rotateBy(currDegrees.inverse()).toDegrees();

    driveTurn /= 180;

    drivetrainSubsystem.drive(
        Vector2.ZERO,
        Math.copySign(
            MathUtil.clamp(0.02 * driveTurn + 0.1 * (old_driveturn - driveTurn), 0.008, 0.05),
            driveTurn),
        false);
    old_driveturn = driveTurn;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(degrees.toDegrees() - currDegrees.toDegrees()) <= 1;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
  }
}