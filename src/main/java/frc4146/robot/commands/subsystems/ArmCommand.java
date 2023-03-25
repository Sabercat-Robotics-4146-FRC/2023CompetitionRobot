package frc4146.robot.commands.subsystems;

import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  private final Arm arm;
  private final Axis extend;
  private final Axis retract;
  private final Axis rotate;

  public ArmCommand(Arm arm, Axis retract, Axis extend, Axis rotate) {
    this.arm = arm;
    this.extend = extend;
    this.retract = retract;
    this.rotate = rotate;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.manually_rotate(-rotate.get() / 1.8);
    arm.manually_extend(extend.get(true) / 2.5 - retract.get(true) / 2.5);
  }

  @Override
  public void end(boolean interrupted) {
    arm.manually_rotate(0);
    arm.manually_extend(0);
  }
}
