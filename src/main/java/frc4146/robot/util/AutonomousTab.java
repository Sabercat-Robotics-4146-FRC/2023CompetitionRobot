package frc4146.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc4146.robot.RobotContainer;

public class AutonomousTab {
  public AutonomousTab(RobotContainer container) {
    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous Chooser");

    SendableChooser<String>[] chooser = container.getAutonomousSelector().getChooser();
    String[] titles = {"Team", "Score", "Engage", "Only Leave"};
    for(int i = 0; i < chooser.length; i++) {
      tab.add(titles[i], chooser[i])
        .withSize(1, 2)
        .withPosition(i, 0);

    }    
  }
}