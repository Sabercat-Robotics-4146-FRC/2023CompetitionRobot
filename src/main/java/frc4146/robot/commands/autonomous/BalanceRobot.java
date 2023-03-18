package frc4146.robot.commands.autonomous;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class BalanceRobot extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Pigeon pigeon;
  // To be tuned
  public double kP = .7 * 0.0125;
  public double kD = 3 * 0.0125 / 40.0;

  public double min_amt = 0.025;
  public double max_amt = 0.1775;

  public double thresh;

  public int stage = 0;

  boolean shortenPos;

  public double past_error;

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon, boolean shortenPos) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
    this.shortenPos = shortenPos;
    // Shuffleboard.getTab("Drivetrain").addNumber("StageB", () -> stage);

    addRequirements(drivetrain);
  }

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this(drivetrain, pigeon, false);
  }

  public void initialize() {

    thresh = 75;
    drivetrain.setMode(true);
    past_error = getError();
    stage = 0;
    drivetrain.resetPose(RigidTransform2.ZERO);
  }

  public void execute() {

    if (stage == 0) {
      double pos = drivetrain.getPose().translation.length;
      if (pos < thresh) drivetrain.drive(new Vector2(0, 0.45), 0);
      else {
        stage += 1;
        drivetrain.resetPose(RigidTransform2.ZERO);
      }
    }
    if (stage == 1) {

      double p = kP * getError();
      double d = kD * getErrorRate();
      double output =
          Math.copySign(
              MathUtil.clamp(
                  Math.abs(p + d),
                  min_amt,
                  max_amt - Math.min(drivetrain.getPose().translation.length / 1350, 0.045)),
              p + d);
      drivetrain.drive(new Vector2(0, output), 0);

      if ((Math.abs(getError()) <= 1)
          && Math.abs(getErrorRate()) < 0.02) { // || getErrorRate() <= -0.25)) {
        stage += 1;
        drivetrain.resetPose(RigidTransform2.ZERO);
      }
    }
  }

  public double getError() {
    return -pigeon.getRoll();
  }

  public double getErrorRate() {
    return getError() - past_error;
  }

  public boolean isFinished() {
    return stage == 2;
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
    drivetrain.setMode(true);
  }
}
