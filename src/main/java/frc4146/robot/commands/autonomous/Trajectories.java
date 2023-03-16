package frc4146.robot.commands.autonomous;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.RobotContainer;
import frc4146.robot.commands.drivetrain.StraightLine;
import frc4146.robot.commands.drivetrain.TurnRobot;
import frc4146.robot.commands.subsystems.ScorePiece;

public class Trajectories {
    SendableChooser<Command> sendableChooser;
    RobotContainer container;
    Map<String, Command> commands;

    public Trajectories(RobotContainer container) {
        this.container = container;
        commands = new HashMap<>();

        commands.put("BlueScoreEngageOne", ScoreEngageOne());
        commands.put("BlueScoreOne", ScoreOne());
        commands.put("BlueEngageOne", EngageOne());
        commands.put("BlueLeaveOne", LeaveOne());
        commands.put("RedEngageThree", ScoreEngageOne());
        commands.put("RedScoreThree", ScoreOne());
        commands.put("RedEngageThree", EngageOne());
        commands.put("RedLeaveThree", LeaveOne());

    }

    public SendableChooser<Command> getSendableChooser() {
        return sendableChooser;
    }

    public Command ScoreEngageOne() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), 55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );

        return command;
    }

    public Command ScoreOne() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150)
        );

        return command;
    }

    public Command EngageOne() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), 55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );

        return command;
    }

    public Command LeaveOne() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150)
        );

        return command;
    }

    public Command ScoreEngageThree() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );

        return command;
    }


    public Command EngageThree() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );

        return command;
    }
}
