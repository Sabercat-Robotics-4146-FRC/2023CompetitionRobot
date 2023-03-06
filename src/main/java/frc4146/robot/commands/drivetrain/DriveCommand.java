package frc4146.robot.commands.drivetrain;

import common.math.Vector2;
import common.robot.input.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;

/*
  Used with joysticks(or some Axis suppliers).
*/
public class DriveCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private Axis forward;
  private Axis strafe;
  private Axis rotation;

  public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis d) {
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = d;

    drivetrainSubsystem = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // double th = 0.01;

    double f = forward.get();
    double s = strafe.get();
    double r = rotation.get(false);
    // if (Math.abs(f) < th) f = 0;
    // if (Math.abs(s) < th) s = 0;
    // if (Math.abs(r) < th) r = 0;

    drivetrainSubsystem.drive(
        new Vector2(
            -Math.copySign(Math.tan(Math.abs(f)) * (Math.sin(Math.abs(f)) + 0.5) / 3.25, f),
            Math.copySign(Math.tan(Math.abs(s)) * (Math.sin(Math.abs(s)) + 0.5) / 3.25, s)),
        Math.copySign(Math.tan(Math.abs(r)) * (Math.sin(Math.abs(r)) + 1) / 40.0, r),
        true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
  }
}
