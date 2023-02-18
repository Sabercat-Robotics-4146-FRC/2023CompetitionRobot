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
    double th = 0.01;
    double f = forward.get(true);
    double s = strafe.get(true);
    double r = rotation.get(true);
    if(Math.abs(f) < th) f = 0;
    if(Math.abs(s) < th) s = 0;
    if(Math.abs(r) < th) r = 0;

    drivetrainSubsystem.drive(
        new Vector2(-f / 2, s / 2), r / 3);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
  }
}
