package org.frcteam4146.common.control;

import org.frcteam4146.common.math.RigidTransform2;
import org.frcteam4146.common.math.Vector2;
import org.frcteam4146.common.util.HolonomicDriveSignal;
import org.frcteam4146.common.util.HolonomicFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolonomicMotionProfiledTrajectoryFollower
    extends TrajectoryFollower<HolonomicDriveSignal> {
  private PidController forwardController;
  private PidController strafeController;
  private PidController rotationController;

  private HolonomicFeedforward feedforward;

  private Trajectory.State lastState = null;
  private Trajectory.State previousState = null;

  private boolean finished = false;

  public HolonomicMotionProfiledTrajectoryFollower(
      PidConstants translationConstants,
      PidConstants rotationConstants,
      HolonomicFeedforward feedforward) {
    this.forwardController = new PidController(translationConstants);
    this.strafeController = new PidController(translationConstants);
    this.rotationController = new PidController(rotationConstants);
    this.rotationController.setContinuous(true);
    this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

    this.feedforward = feedforward;
  }

  @Override
  protected HolonomicDriveSignal calculateDriveSignal(
      RigidTransform2 currentPose,
      Vector2 velocity,
      double rotationalVelocity,
      Trajectory trajectory,
      double time,
      double dt) {
    if (time > trajectory.getDuration()) {
      finished = true;
      return new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
    }

    lastState = trajectory.calculate(time);
    if(previousState == null) {
      previousState = trajectory.calculate(0.0);
    }

    double translationx = ((lastState.getPathState().getPosition().x - previousState.getPathState().getPosition().x) * 50) / 2;
    double translationy = ((lastState.getPathState().getPosition().y - previousState.getPathState().getPosition().y) * 50) / 2;
    double rotation = lastState.getPathState().getRotation().toRadians() - previousState.getPathState().getRotation().toRadians();

    previousState = lastState;

    return new HolonomicDriveSignal(
        new Vector2(
            -translationx, -translationy),
        -rotation,
        true);
  }

  public Trajectory.State getLastState() {
    return lastState;
  }

  @Override
  protected boolean isFinished() {
    return finished;
  }

  @Override
  protected void reset() {
    forwardController.reset();
    strafeController.reset();
    rotationController.reset();

    finished = false;
  }
}
