package frc4146.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc4146.robot.RobotContainer;
import frc4146.robot.commands.autonomous.Trajectories;

import java.util.*;

public class AutonomousSelector {
  private SendableChooser<String>[] pathChooser;
  private Map<String, Command> trajectories;
  private RobotContainer container;

  public AutonomousSelector(RobotContainer container) {
    this.container = container;
    trajectories = new Trajectories(container).getTrajectories();
  }

  public SendableChooser<String>[] getChooser() {
    // configure path chooser
    pathChooser = new SendableChooser[4];
    for(int i = 0; i < 5; i++) pathChooser[i] = new SendableChooser<>();

    pathChooser[0].setDefaultOption("Red", "Red");
    pathChooser[4].setDefaultOption("None", "None");

    for(int i = 1; i < 4; i++) {
      pathChooser[i].setDefaultOption("No", "");
    }
  
    pathChooser[0].addOption("Blue", "Blue");
    pathChooser[1].addOption("Yes", "Score");
    pathChooser[2].addOption("Yes", "Engage");
    pathChooser[3].addOption("Yes", "OnlyLeave");

    pathChooser[4].addOption("One", "One");
    pathChooser[4].addOption("Two", "Two");
    pathChooser[4].addOption("Three", "Three");

    return pathChooser;
  }

  public Command getCommand() {
    String command = "";
    for(SendableChooser<String> c : pathChooser) {
      command += c.getSelected();
    }    

    return trajectories.get(command);
  }
}