package frc4146.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    pathChooser = new SendableChooser[6];
    for (int i = 0; i < 6; i++) pathChooser[i] = new SendableChooser<>();

    pathChooser[0].setDefaultOption("Blue", "Blue");
    pathChooser[5].addOption("Yes", "");

    for (int i = 1; i < 4; i++) {
      pathChooser[i].setDefaultOption("No", "");
    }

    pathChooser[0].addOption("Red", "Red");

    pathChooser[1].addOption("Yes", "Score");
    pathChooser[2].addOption("Yes", "Engage");

    pathChooser[3].addOption("Yes", "Leave");

    pathChooser[4].addOption("Side", "Side");
    pathChooser[4].addOption("Center", "Center");

    pathChooser[5].addOption("No", "0");

    return pathChooser;
  }

  public Command getCommand() {

    String command = "";
    for (SendableChooser<String> c : pathChooser) {
      command += c.getSelected();
    }

    SmartDashboard.putString("command", command);
    if (command.charAt(command.length() - 1) == '0') return new InstantCommand();

    return trajectories.get(command);
  }
}
