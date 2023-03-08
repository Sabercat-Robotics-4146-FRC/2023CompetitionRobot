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
  private final boolean cone;
  private final boolean cube;
  private final boolean intake;
  private final boolean low;
  private final boolean mid;
  private final boolean high;

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
    this.cone = cone.getAsBoolean();
    this.cube = cube.getAsBoolean();
    this.intake = intake.getAsBoolean();
    this.low = low.getAsBoolean();
    this.mid = mid.getAsBoolean();
    this.high = high.getAsBoolean();

    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (intake) {
      arm.setRotationPos(intake_pos[0]);
      arm.setExtensionPos(intake_pos[1]);
    } else if (cone) {
      if (low) {
        arm.setRotationPos(cone_low[0]);
        arm.setExtensionPos(cone_low[1]);
      } else if (mid) {
        arm.setRotationPos(cone_mid[0]);
        arm.setExtensionPos(cone_mid[1]);
      } else if (high) {
        arm.setRotationPos(cone_high[0]);
        arm.setExtensionPos(cone_high[1]);
      }
    } else if (cube) {
      if (low) {
        arm.setRotationPos(cube_low[0]);
        arm.setExtensionPos(cube_low[1]);
      } else if (mid) {
        arm.setRotationPos(cube_mid[0]);
        arm.setExtensionPos(cube_mid[1]);
      } else if (high) {
        arm.setRotationPos(cube_high[0]);
        arm.setExtensionPos(cube_high[1]);
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
