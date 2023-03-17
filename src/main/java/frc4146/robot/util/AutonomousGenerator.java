package frc4146.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousGenerator {
  public static Command getAutonomousCommand(double[][] points) {
    // Assumes the robot starts at (0,0). All values are relative to start position.
    // [(x1,y1),(x2,y2),...]
    double px = 0;
    double py = 0;
    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup();

    for (int i = 0; i < points.length; i++) {
      double x = points[i][0];
      double y = points[i][1];

      double disp_x = x - px;
      double disp_y = y - py;

      double angle = Math.atan2(disp_y, disp_x);
      double distance = Math.hypot(disp_x, disp_y);

      autonomousCommand.addCommands();
    }

    return autonomousCommand;
  }
}
