package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import common.robot.DriverReadout;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  public TalonSRX clawMotor;
  public AnalogPotentiometer pot;
  public DriverReadout _driverInterface = frc4146.robot.RobotContainer.driverInterface;
  public boolean enabled = true;
  public boolean manual_mode = true;
  public boolean clamp = false;

  public double setpoint;

  public Claw() {
    clawMotor = new TalonSRX(ClawConstants.CLAW_ID);
    // clawMotor.setInverted(true);

    clawMotor.setNeutralMode(NeutralMode.Brake);

    Shuffleboard.getTab("Subsystems").addNumber("Claw Position", () -> getPos());
    Shuffleboard.getTab("Subsystems").addNumber("Claw Current", () -> clawMotor.getStatorCurrent());
    Shuffleboard.getTab("DriverReadout").addBoolean("Holding Object?", () -> hasObject());
  }

  public void setClaw(double p) {
    // if (getPos() <= 0.46 && p < 0) p = 0;
    // if (getPos() >= 0.94 && p > 0) p = 0;

    clawMotor.set(ControlMode.PercentOutput, p);
  }

  public void manuallySetClaw(double p) {
    if (manual_mode) {
      if (Math.abs(p) >= 0.01) clamp = false;
      if (!clamp) setClaw(p);
    }
  }

  public void toggleManualMode(boolean m) {
    manual_mode = m;
  }

  public double getPos() {
    return -clawMotor.getSelectedSensorPosition() / 1024;
  }

  public boolean hasObject() {
    return (clawMotor.getStatorCurrent() > 12);
  }

  @Override
  public void periodic() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);
    if (clamp) setClaw(0.75);
  }
}
