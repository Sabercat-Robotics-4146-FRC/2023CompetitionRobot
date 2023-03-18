package frc4146.robot.commands.drivetrain;

import common.math.Vector2;
import common.robot.input.Axis;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  public SlewRateLimiter ff = new SlewRateLimiter(1);
  public SlewRateLimiter sf = new SlewRateLimiter(1);
  public SlewRateLimiter rf = new SlewRateLimiter(1.25);

  public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis d) {
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = d;

    ff.calculate(0);
    sf.calculate(0);

    drivetrainSubsystem = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    double f = forward.get();
    double s = strafe.get();
    double r = rotation.get();

    drivetrainSubsystem.drive(
        new Vector2(
            ff.calculate(
                -Math.copySign(Math.tan(Math.abs(f)) * (Math.sin(Math.abs(f)) + 0.5) / 3.25, f)),
            sf.calculate(
                Math.copySign(Math.tan(Math.abs(s)) * (Math.sin(Math.abs(s)) + 0.5) / 3.25, s))),
        rf.calculate(Math.copySign(Math.tan(Math.abs(r)) * (Math.sin(Math.abs(r)) + 1) / 80.0, r)));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(Vector2.ZERO, 0);
  }
}
