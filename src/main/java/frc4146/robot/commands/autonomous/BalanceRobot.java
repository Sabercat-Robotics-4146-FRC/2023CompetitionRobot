package frc4146.robot.commands.autonomous;

import common.math.Vector2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class BalanceRobot extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  public final Pigeon pigeon;
  // To be tuned
  public double kP = 0.03;
  public double kI = 0;
  public double kD = 0;

  public double inc_roll = 15;

  public PIDController pid;

  public BalanceRobot(DrivetrainSubsystem drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
    this.pid = new PIDController(kP, kI, kD);
    pid.setTolerance(3, 3);
    pid.setIntegratorRange(-0.5, 0.5);
    pid.enableContinuousInput(-180, 180);
  }

  public void initialize() {}

  public void execute() {
    double mag = MathUtil.clamp(pid.calculate(pigeon.getRoll(), 0), -0.5, 0.5);
    drivetrain.drive(
        new Vector2(drivetrain.getPose().rotation.cos, drivetrain.getPose().rotation.sin)
            .scale(mag),
        0);
  }

  public boolean isFinished() {
    return pid.atSetpoint();
  }

  public void end(boolean interrupted) {
    drivetrain.drive(new Vector2(0, 0), 0);
  }
}
