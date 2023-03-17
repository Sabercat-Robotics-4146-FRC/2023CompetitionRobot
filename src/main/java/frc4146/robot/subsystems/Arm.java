package frc4146.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import common.math.MathUtils;
import common.robot.DriverReadout;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4146.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  public DriverReadout _driverInterface = frc4146.robot.RobotContainer.driverInterface;

  public boolean extensionEnabled = true;
  public boolean rotationEnabled = true;

  public TalonFX rotationMotorLeft;
  public TalonFX rotationMotorRight;
  public AnalogPotentiometer pot;

  private double rotPosSetpoint = 0.6;
  public boolean rotPosMode = false;

  public TalonFX extensionMotor;
  public DigitalInput closedLimit;
  public DigitalInput openedLimit;

  private double extPosSetpoint = 0;
  public boolean extPosMode = false;

  public double currentLength;
  public double currentAngle;
  double maxAngle = Math.PI / 3;
  double maxLength = 55;

  public Arm() {
    rotationMotorLeft = new TalonFX(ArmConstants.ROTATION_LEFT_ID);
    rotationMotorLeft.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight = new TalonFX(ArmConstants.ROTATION_RIGHT_ID);
    rotationMotorRight.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight.setInverted(true);

    rotationMotorLeft.setNeutralMode(NeutralMode.Brake);
    rotationMotorRight.setNeutralMode(NeutralMode.Brake);

    pot = new AnalogPotentiometer(ArmConstants.ROTATION_POT_CHANNEl);

    Shuffleboard.getTab("Subsystems").addNumber("Rotation", () -> getRotation());
    Shuffleboard.getTab("Subsystems").addBoolean("RotPosMode", () -> rotPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("RotError", () -> getRotationError());

    // rotPosEntry = Shuffleboard.getTab("Subsystems").add("Rotation SP",
    // rotPosSetpoint).getEntry();

    extensionMotor = new TalonFX(ArmConstants.EXTENSION_ID);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDSlot, ArmConstants.kTimeoutMs);

    closedLimit = new DigitalInput(ArmConstants.LOWER_LIMIT_CHANNEL);
    openedLimit = new DigitalInput(ArmConstants.UPPER_LIMIT_CHANNEL);

    Shuffleboard.getTab("Subsystems").addNumber("Extension", () -> getExtension());
    Shuffleboard.getTab("Subsystems").addBoolean("ExtPosMode", () -> extPosMode);
    Shuffleboard.getTab("Subsystems").addNumber("ExtError", () -> getExtensionError());
    Shuffleboard.getTab("Subsystems").addBoolean("LS Open", () -> openedLimit.get());
    Shuffleboard.getTab("Subsystems").addBoolean("LS Close", () -> closedLimit.get());
    // extPosEntry = Shuffleboard.getTab("Subsystems").add("Extension SP",
    // extPosSetpoint).getEntry();
  }

  public void extend(double p) {
    if (canExtendArm(p) && extensionEnabled) extensionMotor.set(ControlMode.PercentOutput, p);
    else extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  public void manually_extend(double p) {
    if (p > 0) extPosMode = false;
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

  public void toggleExtensionMode(boolean e) {
    extPosMode = e;
  }

  public void rotate(double p) {
    if (canRotateArm(p) && rotationEnabled) {
      rotationMotorLeft.set(ControlMode.PercentOutput, p);
      rotationMotorRight.set(ControlMode.PercentOutput, p);
    } else {
      rotationMotorLeft.set(ControlMode.PercentOutput, 0);
      rotationMotorRight.set(ControlMode.PercentOutput, 0);
    }
  }

  public void manually_rotate(double p) {
    if (p > 0) rotPosMode = false;
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

  public void toggleRotationMode(boolean r) {
    rotPosMode = r;
  }

  public boolean safeToDrive() {
    boolean drive = true;

    currentAngle = -(getRotation() * 2 * Math.PI) + 4.210;
    currentLength =
        (getExtension() * 2 * Math.PI * .75) / (25 / 2)
            + ArmConstants.MIN_LENGTH; // C=2pi*r, gear_ratio=12.5

    if (currentAngle >= maxAngle) {
      drive = false;
      maxLength = 55;
    } else {
      maxLength =
          (ArmConstants.SUPERSTRUCTURE_HEIGHT / Math.cos(currentAngle))
              - 1; // margin of error of 1 inch above ground
    }
    if (currentLength >= maxLength) {
      drive = false;
    }

    return drive;
  }

  @Override
  public void periodic() {

    extensionEnabled = _driverInterface.m_ArmExtSubsystemEnabled.getBoolean(true);
    rotationEnabled = _driverInterface.m_ArmRotSubsystemEnabled.getBoolean(true);

    if (extPosMode) {
      double error = getExtensionError();
      if (Math.abs(error) < 0.125 || (openedLimit.get() && error > 0)) {
        extPosMode = false;
        extend(0);
      } else {
        extend(Math.copySign(MathUtils.clamp(0.05 * Math.abs(error), 0.125, 0.4), error));
      }
    }
    if (rotPosMode) {
      double error = getRotationError();
      if (Math.abs(error) < 0.005) {
        rotPosMode = false;
        rotate(0);
      } else {
        rotate(Math.copySign(MathUtils.clamp(4 * Math.abs(error), 0.15, 0.375), error));
      }
    }
    resetExtensionEncoder();
  }

  public boolean canRotateArm(double p) {
    return (!((getRotation() < ArmConstants.POT_MAX_ROTATION && p < 0)
        || (getRotation() > ArmConstants.POT_MIN_ROTATION && p > 0)));
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
}
