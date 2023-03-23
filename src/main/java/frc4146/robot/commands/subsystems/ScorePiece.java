package frc4146.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4146.robot.subsystems.Arm;
import frc4146.robot.subsystems.Claw;

public class ScorePiece extends SequentialCommandGroup {

  public ScorePiece(Arm arm, Claw claw, String gamepiece, String pos) {
    addCommands(
        Commands.runOnce(
            () -> {
              claw.toggleManualMode(false);
              claw.clamp = true;

            },
            claw),
        new WaitCommand(0.2),
        new PositionPiece(arm, gamepiece, pos),
        new ParallelRaceGroup(
            new RepeatCommand(new InstantCommand(() -> claw.setClaw(-0.85), claw)),
            new WaitCommand(0.3)),
        Commands.runOnce(
            () -> {
              arm.setRotationPos(arm.getRotation() - 0.1);
              arm.toggleRotationMode(true);
            },
            claw,
            arm),
        new WaitCommand(0.25),
        Commands.runOnce(
            () -> {
              arm.setExtensionPos(0);
              arm.toggleExtensionMode(true);
            },
            claw,
            arm),
        new WaitCommand(0.4),
        Commands.runOnce(
            () -> {
              arm.setRotationPos(0.62);
              arm.toggleRotationMode(true);
              claw.toggleManualMode(true);
            }));
  }
}
