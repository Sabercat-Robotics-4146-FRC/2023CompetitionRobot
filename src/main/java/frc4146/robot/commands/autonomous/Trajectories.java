package frc4146.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.RobotContainer;
import frc4146.robot.commands.drivetrain.DriveOverBalance;
import frc4146.robot.commands.drivetrain.StraightLine;
import frc4146.robot.commands.subsystems.ScorePiece;
import frc4146.robot.util.AutonomousGenerator;
import java.util.HashMap;
import java.util.Map;

public class Trajectories {
  SendableChooser<Command> sendableChooser;
  RobotContainer container;
  Map<String, Command> commands;

  AutonomousGenerator autonomousGenerator;

  public Trajectories(RobotContainer container) {
    this.container = container;
    autonomousGenerator = new AutonomousGenerator(this.container);
    commands =
        new HashMap<>() {
          {
            put("BlueScoreSide", ScoreSide());
            put("RedScoreSide", ScoreSide());

            put("BlueScoreLeaveSide", ScoreLeaveSide());
            put("BlueLeaveSide", LeaveSide());
            put("RedScoreLeaveSide", ScoreLeaveSide());
            put("RedLeaveSide", LeaveSide());

            put("BlueScoreEngageSide", BlueScoreEngageSide());
            put("BlueEngageSide", BlueEngageSide());

            put("RedScoreEngageSide", RedScoreEngageSide());
            put("RedEngageSide", RedEngageSide());

            put("BlueScoreEngageCenter", ScoreEngageCenter());
            put("BlueScoreCenter", ScoreCenter());
            put("BlueEngageCenter", EngageCenter());
            put("RedScoreEngageCenter", ScoreEngageCenter());
            put("RedScoreCenter", ScoreCenter());
            put("RedEngageCenter", EngageCenter());

            put("BlueScoreEngageLeaveCenter", ScoreEngageLeaveCenter());
            put("BlueEngageLeaveCenter", EngageLeaveCenter());
            put("RedScoreEngageLeaveCenter", ScoreEngageLeaveCenter());
            put("RedEngageLeaveCenter", EngageLeaveCenter());
          }
        };
  }

  public Map<String, Command> getTrajectories() {
    return commands;
  }

  public Command ScoreSide() {
    return new ScorePiece(
        container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high");
  }

  public Command LeaveSide() {
    return new SequentialCommandGroup(
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -200));
  }

  public Command ScoreLeaveSide() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -200));
  }

  public Command RedEngageSide() {
    return new SequentialCommandGroup(
        autonomousGenerator.getAutonomousCommand(new double[][] {{-140, 0}, {0, -75}}, -80),
        Engage());
  }

  public Command RedScoreEngageSide() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        RedEngageSide());
  }

  public Command BlueEngageSide() {
    return new SequentialCommandGroup(
        autonomousGenerator.getAutonomousCommand(new double[][] {{-140, 0}, {0, 75}}, 80),
        Engage());
  }

  public Command BlueScoreEngageSide() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        BlueEngageSide());
  }

  public Command ScoreCenter() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cube", "high"));
  }

  public Command EngageCenter() {
    return new SequentialCommandGroup(
        autonomousGenerator.getAutonomousCommand(new double[][] {{-10, 0}}, 80),
        new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope()));
  }

  public Command EngageLeaveCenter() {
    return new SequentialCommandGroup(
        autonomousGenerator.getAutonomousCommand(new double[][] {{-10, 0}}, -80),
        new DriveOverBalance(container.getDrivetrainSubsystem(), container.getGyroscope()),
        new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope()));
  }

  public Command ScoreEngageCenter() {
    return new SequentialCommandGroup(ScoreCenter(), EngageCenter());
  }

  public Command ScoreEngageLeaveCenter() {
    return new SequentialCommandGroup(ScoreCenter(), EngageLeaveCenter());
  }

  public Command Engage() {
    return new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope());
  }
}
