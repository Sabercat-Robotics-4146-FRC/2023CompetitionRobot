package frc4146.robot.commands.autonomous;

import common.math.RigidTransform2;
import common.math.Vector2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;
import frc4146.robot.subsystems.Pigeon;

public class BalanceRobotGradient extends CommandBase {
  public DrivetrainSubsystem drivetrain;
  public Pigeon gyro;

  public int stage = 0;
  public ProfiledPIDController pid1;
  public ProfiledPIDController pid2;

  public BalanceRobotGradient(DrivetrainSubsystem drivetrain, Pigeon gyro) {
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    addRequirements(drivetrain);
  }

  public void initialize() {
    stage = 0;
    pid1 = new ProfiledPIDController(0.0125, 0, 3 * 0.0125 / 40.0, new Constraints(0.2, 0.05));
    pid2 = new ProfiledPIDController(0.0125, 0, 3 * 0.0125 / 40.0, new Constraints(0.2, 0.05));
  }

  public void execute() {
    if (stage == 0) {
      if (drivetrain.getPose().translation.length < 75) drivetrain.drive(new Vector2(0, 0.4), 0);
      else {
        stage = 1;
        drivetrain.resetPose(RigidTransform2.ZERO);
      }
    } else if (stage == 1) {
      double roll = -gyro.getRoll();
      double pitch = -gyro.getPitch();

      drivetrain.drive(new Vector2(pid1.calculate(pitch, 0), pid2.calculate(roll, 0)), 0);
    }
  }

  public boolean isFinished() {
    return (stage == 1)
        && (Math.abs(pid1.getPositionError()) <= 1)
        && (Math.abs(pid1.getVelocityError()) <= 0.2)
        && (Math.abs(pid2.getPositionError()) <= 1)
        && (Math.abs(pid2.getVelocityError()) <= 0.2);
  }
}
