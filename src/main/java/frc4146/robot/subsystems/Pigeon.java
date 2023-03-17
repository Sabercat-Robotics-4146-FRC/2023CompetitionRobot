package frc4146.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import common.math.Rotation2;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Pigeon extends WPI_Pigeon2 {

  private Rotation2 adjustmentAngle = Rotation2.ZERO;

  public Pigeon(int id) {
    super(id);
    Shuffleboard.getTab("Drivetrain")
        .addNumber("Pigeon Offset", () -> getAdjustmentAngle().toDegrees());
    // Shuffleboard.getTab("Drivetrain").addNumber("Roll", () -> getRoll());

  }

  public void calibrate() {
    setAdjustmentAngle(getUnadjustedAngle());
  }

  public Rotation2 getAdjustedAngle() {
    return getUnadjustedAngle().rotateBy(adjustmentAngle.inverse());
  }

  public Rotation2 getUnadjustedAngle() {
    return Rotation2.fromDegrees(getAngle());
  }

  public final Rotation2 getAdjustmentAngle() {
    return adjustmentAngle;
  }

  public void setAdjustmentAngle(Rotation2 adjustmentAngle) {
    this.adjustmentAngle = adjustmentAngle;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
  }
}
