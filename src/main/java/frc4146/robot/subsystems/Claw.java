package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import common.robot.DriverReadout;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ClawConstants;

<<<<<<< HEAD
public class Claw implements Subsystem {
  public boolean enabled;

=======
public class Claw extends SubsystemBase {
>>>>>>> Competition
  public TalonSRX clawMotor;
  public AnalogPotentiometer pot;
  public DriverReadout _driverInterface = frc4146.robot.RobotContainer.driverInterface;

  public Claw() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);

    clawMotor = new TalonSRX(ClawConstants.CLAW_ID);
<<<<<<< HEAD
    Shuffleboard.getTab("Subsystems").addNumber("Claw Position", () -> getPos());
    Shuffleboard.getTab("Subsystems").addNumber("Claw Current", () -> clawMotor.getStatorCurrent());
  
    Shuffleboard.getTab("DriverReadout").addBoolean("Holding Object?", () -> hasObject());
  }

  @Override
  public void periodic() {
    enabled = _driverInterface.m_ClawSubsystemEnabled.getBoolean(true);
=======

    clawMotor.config_kP(3,0.3, 30);
    clawMotor.config_kI(3,0, 30);
    clawMotor.config_kD(3,0, 30);


    Shuffleboard.getTab("Subsystems").addNumber("Claw", () -> getPos());
>>>>>>> Competition
  }

  public void manuallySetClaw(double p) {
    if (enabled) {
      clawMotor.set(ControlMode.PercentOutput, p);
    }
  }

  public double getPos() {
    return clawMotor.getSelectedSensorPosition() / 1024;
  }

  public boolean hasObject() {
    return (clawMotor.getStatorCurrent() > 12 && getPos() > 1);
  }
}
