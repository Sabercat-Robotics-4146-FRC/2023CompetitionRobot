package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  public boolean enabled;

  public Claw() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);

    clawMotor = new TalonSRX(ClawConstants.CLAW_ID);
    Shuffleboard.getTab("Subsystems").addNumber("Claw Position", () -> getPos());
    Shuffleboard.getTab("Subsystems").addNumber("Claw Current", () -> clawMotor.getStatorCurrent());
    
    Shuffleboard.getTab("Test Mode").addNumber("Claw Pot", () -> getPos());
  
    Shuffleboard.getTab("DriverReadout").addBoolean("Holding Object?", () -> hasObject());
    
    clawMotor.config_kP(3,0.3, 30);
    clawMotor.config_kI(3,0, 30);
    clawMotor.config_kD(3,0, 30);
  
  }

  @Override
  public void periodic() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);
  }

  /* percent output control mode */
  public void manuallySetClaw(double p) {
    if (enabled) {
      clawMotor.set(ControlMode.PercentOutput, p);
    }
  }

  /* in full potentiometer revolutions */
  public double getPos() {
    return clawMotor.getSelectedSensorPosition() / 1024;
  }

  public boolean hasObject() {
    return (clawMotor.getStatorCurrent() > 12 && getPos() > 1);
  }
}
