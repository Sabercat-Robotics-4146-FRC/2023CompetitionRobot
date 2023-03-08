package frc4146.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {

  private final Arm arm;
  private final double rotation;
  private final double extension;

  public SetArmPosition(Arm arm, double rotation, double extension) {
    this.arm = arm;
    this.rotation = rotation;
    this.extension = extension;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setRotationPos(rotation);
    arm.setExtensionPos(extension);
  }

  @Override
  public void end(boolean interrupted) {
    arm.manually_rotate(0);
    arm.manually_extend(0);
  }
}
