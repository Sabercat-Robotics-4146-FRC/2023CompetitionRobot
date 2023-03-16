package frc4146.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class DriverReadout {

  String tab = "DriverReadout";
  public ShuffleboardLayout primaryLayout;
  public ShuffleboardLayout secondaryLayout;

  public GenericEntry m_ClawSubsystemEnabled;
  public GenericEntry m_ArmExtSubsystemEnabled;
  public GenericEntry m_ArmRotSubsystemEnabled;

  public DriverReadout() {
    ShuffleboardInit();
  }

  private void ShuffleboardInit() {
    // Shuffleboard.getTab(tab).add("Initialized", true).withWidget(BuiltInWidgets.kBooleanBox);
    primaryLayout =
        Shuffleboard.getTab(tab)
            .getLayout("Drive", BuiltInLayouts.kGrid)
            .withSize(5, 4)
            .withPosition(0, 0);

    secondaryLayout =
        Shuffleboard.getTab(tab)
            .getLayout("Arm", BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(5, 0);

    primaryLayout
        .addCamera("Camera", "USB Camera 0", "")
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(1, 0)
        .withSize(3, 3);

    m_ClawSubsystemEnabled = createSubsystemToggle("Claw Subsystem");
    m_ArmExtSubsystemEnabled = createSubsystemToggle("Extension Enabled");
    m_ArmRotSubsystemEnabled = createSubsystemToggle("Rotation Enabled");
  }

  public GenericEntry createSubsystemToggle(String name) {
    return secondaryLayout
        .add(name, true)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(1, 1)
        .getEntry();
  }
}
