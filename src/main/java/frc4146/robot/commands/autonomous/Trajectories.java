package frc4146.robot.commands.autonomous;

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

    public Trajectories(RobotContainer container) {
        this.container = container;
        
        sendableChooser.addOption(
            "BlueOneScoreEngage", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "BlueOneScore", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "BlueOneEngage", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "BlueOneLeave", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "RedThreeScoreEngage", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "RedThreeScore", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "RedThreeEngage", 
            ScoreEngageOne()
        );

        sendableChooser.addOption(
            "RedThreeLeave", 
            ScoreEngageOne()
        );
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
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -100)
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
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -100)
        );

        return command;
    }

    public Command ScoreEngageThree() {
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


    public Command EngageThree() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), 55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );

        return command;
    }

    public Command LeaveThree() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -100)
        );

        return command;
    }
}
