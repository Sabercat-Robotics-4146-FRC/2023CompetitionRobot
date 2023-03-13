package frc4146.robot.commands.gamepiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4146.robot.Constants.ArmConstants;
import frc4146.robot.subsystems.Arm;

public class PositionPiece extends SequentialCommandGroup {
  public PositionPiece(Arm arm, String gamepiece, String pos) {
    double[] setpoints = (double[]) ArmConstants.SETPOINTS.get(gamepiece).get(pos);
    double rot_setpoint = setpoints[0];
    double ext_setpoint = setpoints[1];

    addCommands(
        new InstantCommand(
            () -> {
              arm.setRotationPos(rot_setpoint);
              arm.toggleRotationMode(true);
            },
            arm),
        new ParallelRaceGroup(new WaitUntilCommand(() -> !arm.rotPosMode), new WaitCommand(5)),
        new InstantCommand(
            () -> {
              arm.setExtensionPos(ext_setpoint);
              arm.toggleExtensionMode(true);
            },
            arm),
        new ParallelRaceGroup(new WaitUntilCommand(() -> !arm.extPosMode), new WaitCommand(5)),
        new InstantCommand(() -> arm.toggleExtensionMode(false)));
  }
}
