package frc4146.robot.commands.subsystems;

import static frc4146.robot.Constants.Setpoints.*;

import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4146.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  private final Arm arm;
  private final Axis extend;
  private final Axis retract;
  private final Axis rotate;
  private final Trigger cone;
  private final Trigger cube;
  private final Trigger intake;
  private final Trigger low;
  private final Trigger mid;
  private final Trigger high;

  public ArmCommand(
      Arm arm,
      Axis retract,
      Axis extend,
      Axis rotate,
      Trigger cone,
      Trigger cube,
      Trigger intake,
      Trigger low,
      Trigger mid,
      Trigger high) {
    this.arm = arm;
    this.extend = extend;
    this.retract = retract;
    this.rotate = rotate;
    this.cone = cone;
    this.cube = cube;
    this.intake = intake;
    this.low = low;
    this.mid = mid;
    this.high = high;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (cone.getAsBoolean()) {
      if (intake.getAsBoolean()) {
        new SetArmPosition(arm, intake_cone[0], intake_cone[1]);
      } else if (low.getAsBoolean()) {
        new SetArmPosition(arm, cone_low[0], cone_low[1]);
      } else if (mid.getAsBoolean()) {
        new SetArmPosition(arm, cone_mid[0], cone_mid[1]);
      } else if (high.getAsBoolean()) {
        new SetArmPosition(arm, cone_high[0], cone_high[1]);
      }
    } else if (cube.getAsBoolean()) {
      if (intake.getAsBoolean()) {
        new SetArmPosition(arm, intake_cube[0], intake_cube[1]);
      } else if (low.getAsBoolean()) {
        new SetArmPosition(arm, cube_low[0], cube_low[1]);
      } else if (mid.getAsBoolean()) {
        new SetArmPosition(arm, cube_mid[0], cube_mid[1]);
      } else if (high.getAsBoolean()) {
        new SetArmPosition(arm, cube_high[0], cube_high[1]);
      }
    } else {
      //     if (Math.abs(retract.get()) > 0.10 || Math.abs(extend.get()) > 0.10) {
      arm.manually_rotate(-rotate.get() / 2);
      arm.manually_extend(extend.get() / 4 - retract.get() / 4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.manually_rotate(0);
    arm.manually_extend(0);
  }
}
