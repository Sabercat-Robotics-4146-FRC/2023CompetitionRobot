package frc4146.robot;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc4146.robot.Constants.DriveConstants;
import frc4146.robot.subsystems.Arm;
import frc4146.robot.subsystems.DrivetrainSubsystem;

public class DriverReadout {

  private final RobotContainer container;
  private final DrivetrainSubsystem drivetrain;
  private final Arm arm;

  private final WPI_PigeonIMU gyroscope_sendable;

  public DriverReadout(RobotContainer container, DrivetrainSubsystem drivetrain, Arm arm) {

    this.container = container;
    this.drivetrain = drivetrain;
    this.arm = arm;

    gyroscope_sendable = new WPI_PigeonIMU(DriveConstants.PIGEON_PORT);

    ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

    // Drivetrain
    tab.addBoolean("Drive Enabled", () -> drivetrain.driveFlag).withPosition(0, 0);
    tab.addBoolean("Field Oriented", () -> drivetrain.fieldOriented).withPosition(0, 1);
    tab.add(gyroscope_sendable).withPosition(2, 0);
    tab.addCamera("Intake Camera", "USB Camera 0").withPosition(0, 2).withSize(3, 2);

    // Arm
    tab.addBoolean("Rotation Enabled", () -> arm.rotFlag);
  }
}
