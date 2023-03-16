package frc4146.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.commands.drivetrain.StraightLine;
import frc4146.robot.commands.drivetrain.TurnRobot;
import frc4146.robot.commands.subsystems.ScorePiece;
import frc4146.robot.subsystems.Arm;
import frc4146.robot.subsystems.Claw;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class SimpleTrajectory extends SequentialCommandGroup {
  public SimpleTrajectory(DrivetrainSubsystem drivetrainSubsystem, Pigeon pigeon, Arm arm, Claw claw) {
    addCommands(
        new ScorePiece(arm, claw, "cone", "top"),
        new StraightLine(drivetrainSubsystem, pigeon, -150),
        new TurnRobot(drivetrainSubsystem, pigeon, -90),
        new StraightLine(drivetrainSubsystem, pigeon, 55),
        new BalanceRobot(drivetrainSubsystem, pigeon));
  }
}
