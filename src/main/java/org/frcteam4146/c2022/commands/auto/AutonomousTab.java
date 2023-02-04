package org.frcteam4146.c2022.commands.auto;

import org.frcteam4146.c2022.RobotContainer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutonomousTab {
  public AutonomousTab(RobotContainer container) {
    ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");


    tab.add("Autonomous Mode", container.getAutoSelector().getChooser())
        .withSize(2, 1)
        .withPosition(2, 0);
  }
}
