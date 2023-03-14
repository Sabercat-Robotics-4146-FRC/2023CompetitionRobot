package frc4146.robot.subsystems;

import common.robot.input.Axis;
import common.robot.input.JoystickAxis;
import edu.wpi.first.wpilibj.Joystick;

public class DriveJoysticks {

  private final Joystick joystick_left;
  private final Joystick joystick_right;

  private final Axis leftXAxis;
  private final Axis leftYAxis;

  private final Axis rightXAxis;
  private final Axis rightYAxis;

  public DriveJoysticks(int port_left, int port_right) {
    joystick_left = new Joystick(port_left);
    joystick_right = new Joystick(port_right);

    leftXAxis = new JoystickAxis(joystick_left, 0);

    leftYAxis = new JoystickAxis(joystick_left, 1);
    leftYAxis.setInverted(true);

    rightXAxis = new JoystickAxis(joystick_right, 0);
    rightYAxis = new JoystickAxis(joystick_right, 1);
    rightYAxis.setInverted(true);
  }

  public Axis getLeftXAxis() {
    return leftXAxis;
  }

  public Axis getLeftYAxis() {
    return leftYAxis;
  }

  public Axis getRightXAxis() {
    return rightXAxis;
  }

  public Axis getRightYAxis() {
    return rightYAxis;
  }
}
