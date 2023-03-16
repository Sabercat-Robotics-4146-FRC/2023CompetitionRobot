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
  public double kP = 0.6 * 0.0125;
  public double kI = 0;
  public double kD = 3 * 0.0125 / 40.0;

  public double integrator = 0;

  public double time = 10;

  double min_amt = 0.01;
  double max_amt = 0.135;

  int stage = 0;

  public double past_error;

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
    // Shuffleboard.getTab("Drivetrain").addNumber("StageB", () -> stage);

    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.setMode(true);
    past_error = getError();
    stage = 0;
    integrator = 0;
    drivetrain.resetPose(RigidTransform2.ZERO);
  }

  public void execute() {

    if (stage == 0) {
      double pos = drivetrain.getPose().translation.length;
      if (pos < 40) drivetrain.drive(new Vector2(0, 0.35), 0);
      else {
        stage += 1;
        drivetrain.resetPose(RigidTransform2.ZERO);
      }
    }
    if (stage == 1) {

      integrator += getError();

      double p = kP * getError();
      double i = MathUtil.clamp(kI * integrator, -max_amt / 2, max_amt / 2);
      double d = kD * getErrorRate();
      double output =
          Math.copySign(
              MathUtil.clamp(
                  Math.abs(p + i + d),
                  min_amt,
                  max_amt - Math.min(drivetrain.getPose().translation.length / 1350, 0.0525)),
              p + i + d);
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
