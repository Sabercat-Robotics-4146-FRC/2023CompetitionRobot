package frc4146.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.RobotContainer;

public class AutonomousGenerator {
  public RobotContainer robotContainer;

  public AutonomousGenerator(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  public Command getAutonomousCommand(double[][] points, double final_rotation) {
    // Assumes the robot starts at (0,0). All values are relative to start position.
    // [(x1,y1),(x2,y2),...]
    // What should be the final rotation of the robot relative to its start rotation.
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

      // autonomousCommand.addCommands(
      //    new StraightLineCommand(robotContainer.getDrivetrainSubsystem(), distance, angle));
    }

    return autonomousCommand;
  }
}
