package frc4146.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Pigeon extends WPI_Pigeon2 {

  public Pigeon(int id) {
    super(id);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
  }
}
