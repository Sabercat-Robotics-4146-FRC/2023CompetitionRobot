package org.frcteam4146.common.robot.input;

import edu.wpi.first.wpilibj2.command.button.Button;

public final class NullButton extends Button {
  private boolean value;

  public NullButton() {
    this(false);
  }

  public NullButton(boolean initialValue) {
    value = initialValue;
  }

  public void set(boolean value) {
    this.value = value;
  }
}