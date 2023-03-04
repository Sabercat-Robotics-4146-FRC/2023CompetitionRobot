package frc4146.robot.commands.subsystems;

import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Arm;

public class ManualArmCommand extends CommandBase {
  private final Arm arm;
  private final Axis extend;
  private final Axis retract;
  private final Axis rotate;

  public ManualArmCommand(Arm arm, Axis retract, Axis extend, Axis rotate) {
    this.arm = arm;
    this.extend = extend;
    this.retract = retract;
    this.rotate = rotate;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.manuallyRotateArm(-rotate.get() / 2);
    // deadband 10%
    if (Math.abs(retract.get()) > 0.10 || Math.abs(extend.get()) > 0.10) {
      arm.manuallyExtendArm(extend.get() / 4 - retract.get() / 4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.manuallyExtendArm(0);
    arm.manuallyRotateArm(0);
  }
}
