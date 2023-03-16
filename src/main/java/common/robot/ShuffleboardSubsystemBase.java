package common.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ShuffleboardSubsystemBase extends SubsystemBase {

  String tab; // tab name

  GenericEntry m_kP;
  GenericEntry m_kI;
  GenericEntry m_kD;
  GenericEntry m_setpoint;
  GenericEntry m_pidEnabled;

  public ShuffleboardSubsystemBase(String tabName) {
    ShuffleboardInit(tabName);
    tab = tabName;
  }

  private void ShuffleboardInit(String tabName) {
    Shuffleboard.getTab(tabName).add("Initialized", true).withWidget(BuiltInWidgets.kBooleanBox);

    CameraServer.startAutomaticCapture();
  }

  public GenericEntry NewEntry(String name, WidgetType type, int defaultEntry, int... size) {
    int width = size.length > 0 ? size[0] : 1;
    int height = size.length > 1 ? size[1] : 1;
    return Shuffleboard.getTab(tab)
        .add(name, defaultEntry)
        .withWidget(type)
        .withSize(width, height)
        .getEntry();
  }

  public void PIDConfig(TalonSRX motor) {
    ShuffleboardLayout pidLayout =
        Shuffleboard.getTab(tab).getLayout("PID Config", BuiltInLayouts.kList).withSize(1, 3);

    m_kP =
        pidLayout
            .add("Server kP", 1.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();

    m_kI =
        pidLayout
            .add("Server kI", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();

    m_kD =
        pidLayout
            .add("Server kD", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();

    m_setpoint =
        pidLayout
            .add("Server Setpoint", 2500)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();

    m_pidEnabled =
        pidLayout
            .add("Enable PID", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withSize(1, 1)
            .getEntry();
  }
}
