package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import common.math.MathUtils;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  public TalonFX rotationMotorLeft;
  public TalonFX rotationMotorRight;

  public TalonFX extensionMotor;

  public DigitalInput closedLimit;
  public DigitalInput openedLimit;

  public AnalogPotentiometer pot;

  private double extPosSetpoint = 45;
  public boolean extPosMode = false;

  private double rotPosSetpoint = 0.35;
  public boolean rotPosMode = false;

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

    Shuffleboard.getTab("Subsystems").addNumber("Extension", () -> getExtension());
    Shuffleboard.getTab("Subsystems").addBoolean("ExtPosMode", () -> extPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("ExtError", () -> getExtensionError());

    Shuffleboard.getTab("Subsystems").addNumber("Rotation", () -> getRotation());
    Shuffleboard.getTab("Subsystems").addBoolean("RotPosMode", () -> rotPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("RotError", () -> getRotationError());
  }

  public void extend(double p) {
    if (canExtendArm(p)) extensionMotor.set(ControlMode.PercentOutput, p);
    else extensionMotor.set(ControlMode.PercentOutput, 0);
    extPosMode = false;
  }

  public void setExtensionPos(double encSetpoint) {
    extPosSetpoint = encSetpoint;
  }

  public double getExtensionError() {
    return (extPosSetpoint - getExtension());
  }

  public void toggleExtensionMode() {
    extPosMode = !extPosMode;
  }

  public void rotate(double p) {
    if (canRotateArm(p)) {
      rotationMotorLeft.set(ControlMode.PercentOutput, p);
      rotationMotorRight.set(ControlMode.PercentOutput, p);
    } else {
      rotationMotorLeft.set(ControlMode.PercentOutput, 0);
      rotationMotorRight.set(ControlMode.PercentOutput, 0);
    }
    rotPosMode = false;
  }

  public void setRotationPos(double potSetpoint) {
    rotPosSetpoint = potSetpoint;
  }

  public double getRotationError() {
    return (rotPosSetpoint - getRotation());
  }

  public void toggleRotationMode() {
    rotPosMode = !rotPosMode;
  }

  @Override
  public void periodic() {
    // extPosSetpoint =
    //     Shuffleboard.getTab("Subsystems")
    //         .add("Extension SP", 0)
    //         .getEntry()
    //         .getDouble(extPosSetpoint);
    // rotPosSetpoint =
    //     Shuffleboard.getTab("Subsystems")
    //         .add("Rotation SP", 0)
    //         .getEntry()
    //         .getDouble(rotPosSetpoint);

    if (extPosMode) {
      double error = getExtensionError();
      if (Math.abs(error) < 0.2) {
        extPosMode = false;
        extend(0);
      } else {
        extend(Math.copySign(MathUtils.clamp(0.1 * Math.abs(error), 0.1, 0.4), error));
        extPosMode = true;
      }
    }
    if (rotPosMode) {
      double error = getRotationError();
      if (Math.abs(error) < 0.005) {
        rotPosMode = false;
        rotate(0);
      } else {
        rotate(Math.copySign(MathUtils.clamp(2 * Math.abs(error), 0.1, 0.4), error));
        rotPosMode = true;
      }
    }
  }

  public double getExtension() {
    return extensionMotor.getSelectedSensorPosition() / 2048;
  }

  public double getRotation() {
    return pot.get();
  }

  public boolean canRotateArm(double p) {
    return (!((getRotation() < ArmConstants.POT_MAX_ROTATION && p < 0)
        || (getRotation() > ArmConstants.POT_MIN_ROTATION && p > 0)));
  }

  public boolean canExtendArm(double p) {
    return (!((closedLimit.get() && p < 0) || (openedLimit.get() && p > 0)));
  }
}
