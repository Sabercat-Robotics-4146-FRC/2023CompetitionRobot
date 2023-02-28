package frcteam4146.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frcteam4146.robot.RobotContainer;

public class AutonomousTab {
  public AutonomousTab(RobotContainer container) {
    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous Chooser");

    SendableChooser<String>[] chooser = container.getAutoSelector().getChooser();
    String[] titles = {"Team", "Default", "Pick Up", "Engage", "Only Leave"};
    for(int i = 0; i < chooser.length; i++) {
      tab.add(titles[i], chooser[i])
        .withSize(1, 2)
        .withPosition(i, 0);

    }    
  }
}
