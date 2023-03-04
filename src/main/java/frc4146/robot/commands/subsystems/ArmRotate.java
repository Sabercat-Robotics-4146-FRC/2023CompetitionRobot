package frc4146.robot.commands.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.Arm;
// Not tested

public class ArmRotate extends CommandBase {
  private final Arm arm;
  // To be tuned
  public double kP = 0.3;
  public double kI = 0;
  public double kD = 0;

  public PIDController pid;

  public ArmRotate(Arm arm) {
    this.arm = arm;
    this.pid = new PIDController(kP, kI, kD);
    pid.setTolerance(3, 3);
    pid.setIntegratorRange(-0.5, 0.5);
    pid.enableContinuousInput(-180, 180);
    pid.setSetpoint(-0.55);
  }

  public void initialize() {}

  public void execute() {
    double setpoint = -0.55;
    double mag = MathUtil.clamp(pid.calculate(-arm.getRotation(), setpoint), -0.2, 0.2);
    SmartDashboard.putNumber("Arm Mag", mag);
    SmartDashboard.putNumber("Mag", mag);
    arm.manuallyRotateArm(mag);
  }

  public boolean isFinished() {
    return pid.atSetpoint();
  }

  public void end(boolean interrupted) {
    arm.manuallyRotateArm(0);
  }
}
