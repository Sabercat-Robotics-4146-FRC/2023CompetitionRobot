package frc4146.robot.commands.autonomous;

import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class BalanceRobot extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Pigeon pigeon;
  // To be tuned
  public double kP = -0.17;
  public double kD = -0.01;

  public double time = 10;

  double min_amt = 0.015;
  double max_amt = 0.1;

  public double past_error;

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
  }

  public void initialize() {
    drivetrain.setMode(true);
    past_error = getError();
  }

  public void execute() {

    double p = kP * getError();
    double d = kD * getErrorRate();
    double output = Math.copySign(MathUtil.clamp(Math.abs(p + d), min_amt, max_amt), p + d);

    drivetrain.drive(new Vector2(output, 0), 0);
  }

  public double getError() {
    return pigeon.getRoll() + 1;
  }

  public double getErrorRate() {
    return getError() - past_error;
  }

  public boolean isFinished() {
    return Math.abs(getError()) <= 1 || getErrorRate() <= -0.4;
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
  }
}
