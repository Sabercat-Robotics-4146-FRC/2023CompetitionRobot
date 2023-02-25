package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4146.robot.Constants.ArmConstants;

public class Arm implements Subsystem {
  public TalonFX rotationMotorLeft;
  public TalonFX rotationMotorRight;

  public TalonFX extensionMotor;

  public DigitalInput closedLimit;
  public DigitalInput openedLimit;

  public AnalogPotentiometer pot;

  public PIDController extension_pid;

  public Arm() {
    rotationMotorLeft = new TalonFX(ArmConstants.ROTATION_LEFT_ID);
    rotationMotorLeft.setNeutralMode(NeutralMode.Brake);

    rotationMotorRight = new TalonFX(ArmConstants.ROTATION_RIGHT_ID);
    rotationMotorRight.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight.setInverted(true);

    extensionMotor = new TalonFX(ArmConstants.EXTENSION_ID);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    closedLimit = new DigitalInput(ArmConstants.LOWER_LIMIT_CHANNEL);
    openedLimit = new DigitalInput(ArmConstants.UPPER_LIMIT_CHANNEL);

    pot = new AnalogPotentiometer(ArmConstants.ROTATION_POT_CHANNEl);

    extensionMotor.config_kP(0, 3);
    extensionMotor.config_kI(0, 3);
    extensionMotor.config_kD(0, 3);

    Shuffleboard.getTab("Subsystems").addNumber("Arm Rotation Pot", () -> pot.get());
    Shuffleboard.getTab("Subsystems").addNumber("Arm Extension Encoder", () -> getPos());
  }

  public void manuallyRotateArm(double p) {
    rotationMotorLeft.set(ControlMode.PercentOutput, p);
    rotationMotorRight.set(ControlMode.PercentOutput, p);
  }

  public void manuallyExtendArm(double p) {
    if (!((closedLimit.get() && p < 0) || (openedLimit.get() && p > 0)))
      extensionMotor.set(ControlMode.PercentOutput, p);
    else extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  public void extendArm(double setpoint) {
    extensionMotor.set(ControlMode.Position, setpoint * 2048);
  }

  public double getPos() {
    return extensionMotor.getSelectedSensorPosition() / 2048;
  }
}
