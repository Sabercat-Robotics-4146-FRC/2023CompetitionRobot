package frc4146.robot.commands.subsystems;

import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Claw;

public class ClawCommand extends CommandBase {
  private final Claw claw;
  private final Axis axis;

  private double damping = 1;

  public ClawCommand(Claw claw, Axis axis, boolean testMode) {
    this.claw = claw;
    this.axis = axis;
    if (testMode) {
      damping = 0.2;
    }

    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.manuallySetClaw(axis.get() * damping);
  }

  @Override
  public void end(boolean interrupted) {
    claw.manuallySetClaw(0);
  }
}
