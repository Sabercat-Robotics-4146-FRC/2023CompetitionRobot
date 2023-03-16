package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  public TalonSRX clawMotor;
  public AnalogPotentiometer pot;

  public boolean manual_mode = true;

  public Claw() {
    clawMotor = new TalonSRX(ClawConstants.CLAW_ID);
    Shuffleboard.getTab("Subsystems").addNumber("Claw", () -> getPos());
  }

  public void setClaw(double p) {
    clawMotor.set(ControlMode.PercentOutput, p);
  }

  public void manuallySetClaw(double p) {
    if (manual_mode) setClaw(p);
  }

  public void toggleManualMode(boolean m) {
    manual_mode = m;
  }

  public double getPos() {
    return clawMotor.getSelectedSensorPosition() / 1024;
  }
}
