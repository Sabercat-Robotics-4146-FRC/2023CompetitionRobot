package frc4146.robot.commands.autonomous;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class BalanceRobot extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Pigeon pigeon;
  // To be tuned
  public double kP = -0.015;
  public double kD = -0.0;

  public double time = 10;

  double min_amt = 0.0125;
  double max_amt = 0.135;

  int stage = 0;

  public double past_error;

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
    Shuffleboard.getTab("Drivetrain").addNumber("StageB", () -> stage);

    addRequirements(drivetrain);
  }

  public void initialize() {
    drivetrain.setMode(true);
    past_error = getError();
    stage = 0;
    drivetrain.resetPose(RigidTransform2.ZERO);
  }

  public void execute() {

    if (stage == 0) {
      double pos = drivetrain.getPose().translation.length;
      if (pos < 40) drivetrain.drive(new Vector2(0, -0.35), 0);
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
                  max_amt - Math.min(drivetrain.getPose().translation.length / 1350, 0.05)),
              p + d);
      drivetrain.drive(new Vector2(0, output), 0);

      if ((Math.abs(getError()) <= 1) && Math.abs(getErrorRate()) < 0.02) { // || getErrorRate() <= -0.25)) {
        stage += 1;
        drivetrain.resetPose(RigidTransform2.ZERO);
      }
    }
    if (stage == 2) {
      drivetrain.drive(new Vector2(0, 0.082), 0);
    }
  }

  public double getError() {
    return -pigeon.getYaw();
  }

  public double getErrorRate() {
    return getError() - past_error;
  }

  public boolean isFinished() {
    return stage == 2 && drivetrain.getPose().translation.length >= 0.5;
  }

  public void end(boolean interrupted) {
    drivetrain.drive(Vector2.ZERO, 0);
    drivetrain.setMode(true);
  }
}
