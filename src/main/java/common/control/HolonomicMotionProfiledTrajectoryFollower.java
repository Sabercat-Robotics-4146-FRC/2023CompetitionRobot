package common.control;

import common.math.MathUtils;
import common.math.RigidTransform2;
import common.math.Vector2;
import common.util.HolonomicDriveSignal;
import common.util.HolonomicFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolonomicMotionProfiledTrajectoryFollower
    extends TrajectoryFollower<HolonomicDriveSignal> {
  private PidController forwardController;
  private PidController strafeController;
  private PidController rotationController;

  private Trajectory.State lastState = null;
  private Trajectory.State previousState = null;

  private boolean finished = false;

  private double lastTime = 0;

  private double scale;
  private double maxSpeed = .5;
  private double reduction = 0;

  private double lastTranslationx = 0;
  private double lastTranslationy = 0;

  public HolonomicMotionProfiledTrajectoryFollower(
      PidConstants translationConstants,
      PidConstants rotationConstants,
      HolonomicFeedforward feedforward) {
    this.forwardController = new PidController(translationConstants);
    this.strafeController = new PidController(translationConstants);
    this.rotationController = new PidController(rotationConstants);
    this.rotationController.setContinuous(true);
    this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

    this.scale = 1;
  }

  @Override
  protected HolonomicDriveSignal calculateDriveSignal(
      RigidTransform2 currentPose,
      Vector2 velocity,
      double rotationalVelocity,
      Trajectory trajectory,
      double time,
      double dt) {

    if(lastTime == 0) lastTime = time;

    time -= reduction;

    if (time > trajectory.getDuration()) {
      finished = true;
      return new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
    }

    lastState = trajectory.calculate(time);
    if(previousState == null) {
      previousState = trajectory.calculate(0.0);
    }

    double translationx = (lastState.getPathState().getPosition().x - previousState.getPathState().getPosition().x) * 100;
    double translationy = (lastState.getPathState().getPosition().y - previousState.getPathState().getPosition().y) * 100;


    if(translationx > maxSpeed || translationx < -maxSpeed) {
      scale = Math.abs(translationx / maxSpeed);
      translationx = maxSpeed * (translationx/Math.abs(translationx));
      translationy /= scale;
      reduction += (time - lastTime) - (time - lastTime) / scale;
    }

    if(translationy > maxSpeed || translationy < -maxSpeed) {
      scale = Math.abs(translationy / maxSpeed);
      translationy = maxSpeed * (translationy/Math.abs(translationy));
      translationx /= scale;
      reduction += (time - lastTime) - (time - lastTime) / scale;
    }

    if(Math.abs(translationx - lastTranslationx) > maxSpeed/1.25) {
      translationx = lastTranslationx;
    }

    if(Math.abs(translationy - lastTranslationy) > maxSpeed/1.25) {
      translationy = lastTranslationy;
    }



    SmartDashboard.putNumber("X", translationx);
    SmartDashboard.putString("TESTTTTT", time + reduction + " " + trajectory.getDuration());
    
    lastTime = time;
    previousState = lastState;
    lastTranslationx = translationx;
    lastTranslationy = translationy;

    return new HolonomicDriveSignal(
        new Vector2(
           0, 0),
        0,
        false);
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
