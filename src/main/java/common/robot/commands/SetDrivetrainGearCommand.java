package common.robot.commands;

import common.robot.subsystems.ShiftingTankDrivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

@Deprecated
public class SetDrivetrainGearCommand extends CommandBase {
  private final ShiftingTankDrivetrain drivetrain;
  private final boolean highGear;

  public SetDrivetrainGearCommand(ShiftingTankDrivetrain drivetrain, boolean highGear) {
    this.drivetrain = drivetrain;
    this.highGear = highGear;
  }

  @Override
  public void initialize() {
    drivetrain.setHighGear(highGear);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
