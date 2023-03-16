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
import frc4146.robot.subsystems.Pigeon;

public class Trajectories {
    SendableChooser<Command> sendableChooser;
    RobotContainer container;
    Map<String, Command> commands;

    public Trajectories(RobotContainer container) {
        this.container = container;
        commands = new HashMap<>() { {
            put("BlueScoreEngageOne", ScoreEngageOne());
            put("BlueScoreOne", ScoreOne());
            put("BlueEngageOne", EngageOne());
            put("BlueLeaveOne", LeaveOne());

            put("RedScoreEngageThree", ScoreEngageOne());
            put("RedScoreThree", ScoreOne());
            put("RedEngageThree", EngageOne());
            put("RedLeaveThree", LeaveOne());


            put("BlueScoreEngageThree", ScoreEngageThree());
            put("BlueScoreThree", ScoreOne());
            put("BlueEngageThree", EngageThree());
            put("BlueLeaveThree", LeaveOne());
            
            put("RedEngageOne", ScoreEngageThree());
            put("RedScoreOne", ScoreOne());
            put("RedEngageOne", EngageThree());
            put("RedLeaveOne", LeaveOne());


            put("BlueScoreEngageTwo", ScoreEngageTwo());
            put("BlueScoreTwo", ScoreTwo());
            put("BlueEngageTwo", EngageTwo());
            
            put("RedScoreEngageTwo", ScoreEngageTwo());
            put("RedScoreTwo", ScoreTwo());
            put("RedEngageTwo", EngageTwo());


            put("BlueEngageNone", Engage());
            put("RedEngageNone", Engage());

        }};
    }

    public Map<String, Command> getTrajectories() {
        return commands;
    }

    public Command ScoreEngageOne() {
        //Confirmed
        return new SequentialCommandGroup(
            ScoreOne(),
            EngageOne()
        );
    }

    public Command ScoreOne() {
        //Confirmed
        return new SequentialCommandGroup(
            new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cone", "high"),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150)
        );
    }

    public Command EngageOne() {
        //Confirmed
        return new SequentialCommandGroup(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), 55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );
    }

    public Command LeaveOne() {
        //Confirmed
        return new SequentialCommandGroup(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150)
        );
    }

    public Command ScoreEngageTwo() {
        //Confirmed
        return new SequentialCommandGroup(
            ScoreTwo(),
            EngageTwo()
        );
    }

    public Command ScoreTwo() {
        //Confirmed
        return new SequentialCommandGroup(
            new ScorePiece(container.getArmSubsystem(), container.getClawSubsystem(), "cube", "high")
        );
    }

    public Command EngageTwo() {
        //Confirmed
        return new SequentialCommandGroup(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -50),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), 90),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );
    }

    public Command ScoreEngageThree() {
        return new SequentialCommandGroup(
            ScoreOne(),
            EngageThree()
        );
    }

    public Command EngageThree() {
        return new SequentialCommandGroup(
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -150),
            new TurnRobot(container.getDrivetrainSubsystem(), container.getGyroscope(), -90),
            new StraightLine(container.getDrivetrainSubsystem(), container.getGyroscope(), -55),
            new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope())
        );
    }

    public Command Engage() {
        return new BalanceRobot(container.getDrivetrainSubsystem(), container.getGyroscope());
    }
}
