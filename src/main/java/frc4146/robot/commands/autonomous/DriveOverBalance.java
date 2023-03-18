package frc4146.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4146.robot.commands.drivetrain.StraightLine;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class DriveOverBalance extends SequentialCommandGroup {
  public DriveOverBalance(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    addCommands(
        new ParallelRaceGroup(
            new StraightLine(drivetrain, pigeon, -4000),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> pigeon.getPitch() >= 5), // Driving backwards
                new WaitCommand(3.5))));
  }
}
