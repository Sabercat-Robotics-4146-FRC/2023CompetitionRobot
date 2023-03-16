package common.robot;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc4146.robot.Constants.DriveConstants;

public class DriverReadout {

    String tab = "Driver";
    public ShuffleboardLayout primaryLayout;
    public ShuffleboardLayout secondaryLayout;

    public GenericEntry m_ClawSubsystemEnabled;
    public GenericEntry m_ArmExtSubsystemEnabled;
    public GenericEntry m_ArmRotSubsystemEnabled;

    public WPI_PigeonIMU gyroscope_Sendable;

    public DriverReadout() {
        ShuffleboardInit();
    }

    private void ShuffleboardInit() {
        // Shuffleboard.getTab(tab).add("Initialized", true).withWidget(BuiltInWidgets.kBooleanBox);
        primaryLayout =
            Shuffleboard.getTab(tab)
                .getLayout("Driver", BuiltInLayouts.kGrid)
                .withSize(5, 4)
                .withPosition(0, 0);

        secondaryLayout =
            Shuffleboard.getTab(tab)
                .getLayout("Arm", BuiltInLayouts.kGrid)
                .withSize(3, 4)
                .withPosition(5, 0);

        primaryLayout
            .addCamera("Cam", "USB Camera 0", "")
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(1, 0);

        m_ClawSubsystemEnabled = createSubsystemToggle("Claw Subsystem");
        m_ArmExtSubsystemEnabled = createSubsystemToggle("Extension Enabled");
        m_ArmRotSubsystemEnabled = createSubsystemToggle("Rotation Enabled");

        gyroscope_Sendable = new WPI_PigeonIMU(DriveConstants.PIGEON_PORT);
        primaryLayout.add(gyroscope_Sendable).withPosition(0, 0);
    }

    public GenericEntry createSubsystemToggle(String name) {
        return secondaryLayout
            .add(name, true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withSize(1, 1)
            .getEntry();
    }
}
