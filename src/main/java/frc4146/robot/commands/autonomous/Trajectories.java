package frc4146.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.RobotContainer;
import frc4146.robot.commands.drivetrain.DriveOverBalance;
import frc4146.robot.commands.drivetrain.StraightLine;
import frc4146.robot.commands.drivetrain.TurnRobot;
import frc4146.robot.commands.subsystems.ScorePiece;
import java.util.HashMap;
import java.util.Map;

public class Trajectories {
  SendableChooser<Command> sendableChooser;
  RobotContainer container;
  Map<String, Command> commands;

  public Trajectories(RobotContainer container) {
    this.container = container;
    commands =
        new HashMap<>() {
          {
            put("BlueScoreEngageOne", ScoreEngageOne());
            put("BlueScoreOne", ScoreOne());
            put("BlueEngageOne", EngageOne());
            put("BlueLeaveOne", Leave());

            put("RedScoreEngageThree", ScoreEngageOne());
            put("RedScoreThree", ScoreOne());
            put("RedEngageThree", EngageOne());
            put("RedLeaveThree", Leave());

            put("BlueScoreEngageThree", ScoreEngageThree());
            put("BlueScoreThree", ScoreOne());
            put("BlueEngageThree", EngageThree());
            put("BlueLeaveThree", Leave());

            put("RedScoreEngageOne", ScoreEngageThree());
            put("RedScoreOne", ScoreOne());
            put("RedEngageOne", EngageThree());
            put("RedLeaveOne", Leave());

            put("BlueScoreEngageTwo", ScoreEngageTwo());
            put("BlueScoreTwo", ScoreTwo());
            put("BlueEngageTwo", EngageTwo());

            put("RedScoreEngageTwo", ScoreLeaveEngageTwo());
            put("RedScoreTwo", ScoreTwo());
            put("RedEngageTwo", EngageTwo());

            put("BlueEngageNone", Engage());
            put("RedEngageNone", Engage());
          }
        };
  }

  public Map<String, Command> getTrajectories() {
    return commands;
  }

  public Command ScoreOne() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -200));
  }

  public Command EngageOne() {
    return new SequentialCommandGroup(
        Leave(),
        new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), 55),
        Engage());
  }

  public Command ScoreEngageOne() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        EngageOne());
  }

  public Command EngageThree() {
    return new SequentialCommandGroup(
        Leave(),
        new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -55),
        Engage());
  }

  public Command ScoreEngageThree() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
        EngageThree());
  }

  public Command ScoreTwo() {
    return new SequentialCommandGroup(
        new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cube", "high"));
  }

  public Command EngageTwo() {
    return new SequentialCommandGroup(
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -10),
        new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), 90),
        new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope()));
  }

  public Command ScoreEngageTwo() {
    return new SequentialCommandGroup(ScoreTwo(), EngageTwo());
  }

  public Command ScoreLeaveEngageTwo() {
    return new SequentialCommandGroup(
        ScoreTwo(),
        new DriveOverBalance(container.getDrivetrainSubsystem(), container.getGyroscope()),
        new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -80),
        new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope()));
  }

  public Command Engage() {
    return new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope());
  }

  public Command Leave() {
    return new SequentialCommandGroup(
        new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -200));
  }
}
