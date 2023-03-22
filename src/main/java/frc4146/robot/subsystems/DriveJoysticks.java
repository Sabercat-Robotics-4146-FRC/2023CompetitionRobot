package frc4146.robot.subsystems;

import common.robot.input.Axis;
import common.robot.input.JoystickAxis;
import edu.wpi.first.wpilibj.Joystick;

public class DriveJoysticks {

  private final Joystick joystick;

  private final Axis xAxis;
  private final Axis yAxis;
  private final Axis rAxis;

  public DriveJoysticks(int port) {
    joystick = new Joystick(port);

    xAxis = new JoystickAxis(joystick, 0);
    yAxis = new JoystickAxis(joystick, 1);
    rAxis = new JoystickAxis(joystick, 2);

    yAxis.setInverted(true);
  }

  public Axis getXAxis() {
    return xAxis;
  }

  public Axis getYAxis() {
    return yAxis;
  }

  public Axis getRAxis() {
    return rAxis;
  }
}
