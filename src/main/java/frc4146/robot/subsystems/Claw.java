package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import common.robot.DriverReadout;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4146.robot.Constants.ClawConstants;

public class Claw implements Subsystem {
  public boolean enabled;

  public TalonSRX clawMotor;
  public AnalogPotentiometer pot;
  public DriverReadout _driverInterface = frc4146.robot.RobotContainer.driverInterface;

  public Claw() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);

    clawMotor = new TalonSRX(ClawConstants.CLAW_ID);
    Shuffleboard.getTab("Subsystems").addNumber("Claw", () -> getPos());
  }

  @Override
  public void periodic() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);
  }

  public void manuallySetClaw(double p) {
    if (enabled) {
      clawMotor.set(ControlMode.PercentOutput, p);
    }
  }

  public double getPos() {
    return clawMotor.getSelectedSensorPosition() / 1024;
  }
}
