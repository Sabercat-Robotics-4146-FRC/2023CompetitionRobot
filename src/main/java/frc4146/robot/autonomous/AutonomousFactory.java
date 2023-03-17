package frc4146.robot.autonomous;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4146.robot.RobotContainer;
import frc4146.robot.commands.FollowTrajectoryCommand;
import frc4146.robot.subsystems.DrivetrainSubsystem;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import common.control.Path;
import common.control.SplinePathBuilder;
import common.control.Trajectory;
import common.math.Rotation2;
import common.math.Vector2;

public class AutonomousFactory {
  RobotContainer container;

  public AutonomousFactory(RobotContainer container) {
    this.container = container;
  }

  /** Creates an autonomous path. */
  public SequentialCommandGroup createPath(String pathDirectory) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    File root = new File(pathDirectory);
    // create an object for trajectories folder
    File waypointsFile = root.listFiles((dir, name) -> name.contains(".path"))[0];

    // create the list of states from .json file
    List<List<State>> states = parseJson(waypointsFile.toPath());

    // Create trajectories from
    List<Trajectory> trajectories = getTrajectories(states);

    // compile a command that incorperates trajectory and subsystem commands
    for (int i = 0; i < trajectories.size(); i++) {
      Trajectory t = trajectories.get(i);

      command.addCommands(
        new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), t)
      );
    }

    return command;
  }

  public List<SequentialCommandGroup> getMidCommands(List<List<State>> values, List<List<Double>> times) {
    List<SequentialCommandGroup> commands = new ArrayList<>();

    for(int j = 0; j < values.size(); j++) {
      SequentialCommandGroup c = new SequentialCommandGroup();
      List<State> list = values.get(j);
      List<Double> ntimes = times.get(j);
      double lastTime = 0.0;

      for (int i = 0; i < list.size() - 1; i++) {
        State state = list.get(i);
        double time = ntimes.get(i);
        c.addCommands(getCommand(state.action).withTimeout(time - lastTime));
        lastTime = time;
      }
      commands.add(c);
    }

    return commands;
  }

  public List<SequentialCommandGroup> getEndCommands(List<List<State>> list) {
    List<SequentialCommandGroup> commands = new ArrayList<>();

    for (List<State> states : list) {
      SequentialCommandGroup c =
          new SequentialCommandGroup(getCommand(states.get(states.size() - 1).action));
      commands.add(c);
    }

    return commands;
  }

  public List<Trajectory> getTrajectories(List<List<State>> waypoints) {
    List<Trajectory> trajectories = new ArrayList<>();

    for (List<State> list : waypoints) {
      // filter .json files only
      Trajectory trajectory;
      if(list.size() == 0) continue;
      //try {
        SplinePathBuilder splinePath = 
          new SplinePathBuilder(
            new Vector2(list.get(0).x, list.get(0).y),
            new Rotation2(Math.cos(list.get(0).heading), Math.sin(list.get(0).heading), true), 
            new Rotation2(Math.cos(list.get(0).heading), Math.sin(list.get(0).heading), true));

        for(int j = 0; j < list.size()-1; j++) {
          State waypoint = list.get(j);
          State waypoint2 = list.get(j+1);
          Vector2 start = new Vector2(waypoint.x, waypoint.y);
          Vector2 v1 = new Vector2(waypoint.tx, waypoint.ty);
          Vector2 v2 = new Vector2(waypoint2.tx, waypoint2.ty);
          Vector2 end = new Vector2(waypoint2.x, waypoint2.y);
          Rotation2 r = new Rotation2(Math.cos(waypoint.heading), Math.sin(waypoint.heading), true);

          splinePath.quinticHermite(start, v1, end, v2, r);
        }
        
        Path path = splinePath.build();

        trajectory = new Trajectory(path, DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0.1);
        trajectories.add(trajectory);
    }

    return trajectories;
  }

  public List<List<State>> parseJson(java.nio.file.Path path) {

    // read JSON file
    String json = "";
    try {
      json = Files.readString(path, StandardCharsets.UTF_8);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // Convert JSON file into an ArrayList of type State
    // This creates an array list of all the waypoints, with
    // State representing the values within each waypoint.
    Type type = new TypeToken<List<State>>() {}.getType();
    List<State> list = new Gson().fromJson(json, type);

    // Map waypoints such that they corrospond to their respective
    // trajectories.
    List<List<State>> dividedLists = new ArrayList<>();
    List<State> temp = new ArrayList<>();
    list.forEach(
        l -> {
          temp.add(l);
          if (l.isBreak) {
            dividedLists.add(temp);
            temp.clear();
            // temp.add(l);
          }
        });
    dividedLists.add(temp);

    return dividedLists;
  }

  // this is just a skeleton method for now. When all commands are
  // implemented, add them to this method

  private Command getCommand(String name) {
    // return
    //   switch (name) {
    //     default -> new SequentialCommandGroup();
    // }

    return new SequentialCommandGroup();
  }

  public static class State {
    public double x;
    public double y;
    private double tx;
    private double ty;
    private double heading;
    private boolean isBreak;
    private String action;

    public String toString() {
      return "x = "
          + x
          + " y = "
          + y
          + "tx = "
          + tx 
          + "ty = "
          + ty
          + "heading = "
          + heading
          + "break = "
          + isBreak
          + " action = "
          + action;
    }
  }
}