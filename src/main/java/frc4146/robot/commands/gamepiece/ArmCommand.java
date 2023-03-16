package frc4146.robot.commands.gamepiece;

import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  private final Arm arm;
  private final Axis extend;
  private final Axis retract;
  private final Axis rotate;

  private double damping = 1;

  public ArmCommand(Arm arm, Axis retract, Axis extend, Axis rotate, boolean testMode) {
    this.arm = arm;
    this.extend = extend;
    this.retract = retract;
    this.rotate = rotate;
    if (testMode) {
      damping = 0.2;
    }

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.manually_rotate(-rotate.get(true) * damping);
    arm.manually_extend(extend.get(true) * damping / 2 - retract.get(true) * damping / 2);
  }

  @Override
  public void end(boolean interrupted) {
    arm.manually_rotate(0);
    arm.manually_extend(0);
  }
}
