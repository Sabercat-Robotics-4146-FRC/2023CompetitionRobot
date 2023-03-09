package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import common.math.MathUtils;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  public TalonFX rotationMotorLeft;
  public TalonFX rotationMotorRight;
  public AnalogPotentiometer pot;

  private double rotPosSetpoint = 0.6;
  private GenericEntry rotPosEntry;
  public boolean rotPosMode = false;
  public boolean rotFlag = true;

  public TalonFX extensionMotor;
  public DigitalInput closedLimit;
  public DigitalInput openedLimit;

  private double extPosSetpoint = 0;
  private GenericEntry extPosEntry;
  public boolean extPosMode = false;
  public boolean extFlag = true;

  public Arm() {
    rotationMotorLeft = new TalonFX(ArmConstants.ROTATION_LEFT_ID);
    rotationMotorLeft.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight = new TalonFX(ArmConstants.ROTATION_RIGHT_ID);
    rotationMotorRight.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight.setInverted(true);

    pot = new AnalogPotentiometer(ArmConstants.ROTATION_POT_CHANNEl);

    Shuffleboard.getTab("Subsystems").addNumber("Rotation", () -> getRotation());
    Shuffleboard.getTab("Subsystems").addBoolean("RotPosMode", () -> rotPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("RotError", () -> getRotationError());

    rotPosEntry = Shuffleboard.getTab("Subsystems").add("Rotation SP", rotPosSetpoint).getEntry();

    extensionMotor = new TalonFX(ArmConstants.EXTENSION_ID);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDSlot, ArmConstants.kTimeoutMs);
    extensionMotor.configFeedbackNotContinuous(false, ArmConstants.kTimeoutMs);

    closedLimit = new DigitalInput(ArmConstants.CLOSED_LIMIT_CHANNEL);
    openedLimit = new DigitalInput(ArmConstants.OPEN_LIMIT_CHANNEL);

    Shuffleboard.getTab("Subsystems").addNumber("Extension", () -> getExtension());
    Shuffleboard.getTab("Subsystems").addBoolean("ExtPosMode", () -> extPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("ExtError", () -> getExtensionError());

    extPosEntry = Shuffleboard.getTab("Subsystems").add("Extension SP", extPosSetpoint).getEntry();
  }

  public void extend(double p) {
    if (canExtendArm(p)) extensionMotor.set(ControlMode.PercentOutput, p);
    else extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  public void manually_extend(double p) {
    if (!extPosMode) extend(p);
  }

  public double getExtension() {
    return extensionMotor.getSelectedSensorPosition() / 2048;
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
  }

  public void manually_rotate(double p) {
    if (!rotPosMode) rotate(p);
  }

  public double getRotation() {
    return pot.get();
  }

  public void setRotationPos(double potSetpoint) {
    rotPosSetpoint = potSetpoint;
  }

  public double getRotationError() {
    return rotPosSetpoint - getRotation();
  }

  public void toggleRotationMode() {
    rotPosMode = !rotPosMode;
  }

  @Override
  public void periodic() {

    extPosSetpoint = extPosEntry.getDouble(extPosSetpoint);
    rotPosSetpoint = rotPosEntry.getDouble(rotPosSetpoint);

    if (extPosMode) {
      double error = getExtensionError();
      if (Math.abs(error) < 0.1) {
        extPosMode = false;
        extend(0);
      } else {
        extend(Math.copySign(MathUtils.clamp(0.15 * Math.abs(error), 0.05, 0.2), error));
      }
    }
    if (rotPosMode) {
      double error = getRotationError();
      if (Math.abs(error) < 0.005) {
        rotPosMode = false;
        rotate(0);
      } else {
        rotate(Math.copySign(MathUtils.clamp(2 * Math.abs(error), 0.1, 0.15), error));
      }
    }

    resetExtensionEncoder();

    /*changeRotFlag();
    changeExtFlag();*/
  }

  public boolean canRotateArm(double p) {
    return (!((getRotation() < ArmConstants.POT_MAX_ROTATION && p < 0)
            || (getRotation() > ArmConstants.POT_MIN_ROTATION && p > 0))
        && (rotFlag));
  }

  public boolean canExtendArm(double p) {
    return (!((closedLimit.get() && p < 0) || (openedLimit.get() && p > 0)));
  }

  public void resetExtensionEncoder() {
    if (closedLimit.get()) {
      extensionMotor.setSelectedSensorPosition(
          0); // make sure arm isn't traveling too far past limit switch
    }
  }

  /*public void changeRotFlag() {
    if (Shuffleboard.getTab("Driver Readout").)
  }*/
}
