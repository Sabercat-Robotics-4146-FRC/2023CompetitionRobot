package common.robot.commands;

import common.robot.subsystems.HolonomicDrivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

@Deprecated
public class ZeroFieldOrientedCommand extends CommandBase {
  private final HolonomicDrivetrain drivetrain;

  public ZeroFieldOrientedCommand(HolonomicDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    drivetrain.getGyroscope().setAdjustmentAngle(drivetrain.getGyroscope().getUnadjustedAngle());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
