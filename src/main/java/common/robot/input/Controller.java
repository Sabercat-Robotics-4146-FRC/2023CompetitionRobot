package common.robot.input;


import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An interface for easily implementing gamepads as an input source.
 *
 * @author Jacob Bublitz
 * @see XboxController
 * @since 1.0
 */
public abstract class Controller {
  public abstract Axis getLeftTriggerAxis();

  public abstract Axis getLeftXAxis();

  public abstract Axis getLeftYAxis();

  public abstract Axis getRightTriggerAxis();

  public abstract Axis getRightXAxis();

  public abstract Axis getRightYAxis();

  /**
   * Get the A button of the controller.
   *
   * @return The A button
   * @since 1.0
   */
  public abstract Trigger getAButton();

  /**
   * Get the B button of the controller.
   *
   * @return The B button
   * @since 1.0
   */
  public abstract Trigger getBButton();

  /**
   * Get the X button of the controller.
   *
   * @return The X button
   * @since 1.0
   */
  public abstract Trigger getXButton();

  /**
   * Get the Y button of the controller.
   *
   * @return The Y button
   * @since 1.0
   */
  public abstract Trigger getYButton();

  /**
   * Get the left bumper button of the controller.
   *
   * @return The left bumper button
   * @since 1.0
   */
  public abstract Trigger getLeftBumperButton();

  /**
   * Get the right bumper button of the controller.
   *
   * @return The right bumper button
   * @since 1.0
   */
  public abstract Trigger getRightBumperButton();

  /**
   * Get the back button of the controller.
   *
   * @return The back button
   * @since 1.0
   */
  public abstract Trigger getBackButton();

  /**
   * Get the start button of the controller.
   *
   * @return The start button
   * @since 1.0
   */
  public abstract Trigger getStartButton();

  /**
   * Get the left joystick button of the controller.
   *
   * @return The left joystick button
   * @since 1.0
   */
  public abstract Trigger getLeftJoystickButton();

  /**
   * Get the right joystick button of the controller.
   *
   * @return The right joystick button
   * @since 1.0
   */
  public abstract Trigger getRightJoystickButton();

  /**
   * Get a D-Pad button of the controller.
   *
   * @param direction The direction of the D-Pad button
   * @return The D-Pad button of the specified direction
   * @since 1.0
   */
  public abstract Trigger getDPadButton(DPadButton.Direction direction);
}
