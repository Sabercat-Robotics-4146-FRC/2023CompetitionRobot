package frcteam4146.robot.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frcteam4146.robot.RobotContainer;

import java.io.File;
import java.util.*;

public class AutonomousSelector {
  private SendableChooser<String>[] pathChooser;
  private Map<String, String> paths;
  private RobotContainer container;

  public AutonomousSelector(RobotContainer container) {
    this.container = container;

    File directory =
        Filesystem.getDeployDirectory().listFiles((dir, name) -> name.contains("output"))[0];

    // List of all autonomous paths folders
    paths = new HashMap<String, String>();
    for (File f : directory.listFiles()) {
      if(!f.isDirectory()) continue;

      String fileName = f.getName();
      String filePath = f.getPath();

      paths.putIfAbsent(fileName, filePath);
    }
  }

  public SendableChooser<String>[] getChooser() {
    // configure path chooser
    pathChooser = new SendableChooser[5];
    for(int i = 0; i < 5; i++) pathChooser[i] = new SendableChooser<>();

    pathChooser[0].addOption("Blue", "Blue");
    pathChooser[0].setDefaultOption("Red", "Red");

    pathChooser[1].addOption("Yes", "Default");
    pathChooser[2].addOption("Yes", "PickUp");
    pathChooser[3].addOption("Yes", "Engage");
    pathChooser[4].addOption("Yes", "OnlyLeave");

    for(int i = 1; i < 5; i++) {
      pathChooser[i].setDefaultOption("No", "");
    }

    return pathChooser;
  }

  public Command getCommand() {
    AutonomousFactory auto = new AutonomousFactory(container);
    SequentialCommandGroup command = new SequentialCommandGroup();

    String path = "";
    for(SendableChooser<String> c : pathChooser) {
      path += c.getSelected();
    }

    if(path.contains("OnlyLeave")) path = "OnlyLeave";

    command.addCommands(
      auto.createPath(paths.get(path)) // creates path command
    );

    return command;
  }
}
