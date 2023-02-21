package org.frcteam4146.common.control;

import java.util.*;
import org.frcteam4146.common.math.Rotation2;
import org.frcteam4146.common.math.Vector2;
import org.frcteam4146.common.math.spline.CubicBezierSpline;
import org.frcteam4146.common.math.spline.CubicHermiteSpline;
import org.frcteam4146.common.math.spline.QuinticHermiteSpline;
import org.frcteam4146.common.math.spline.Spline;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SplinePathBuilder {
  private List<PathSegment> segmentList = new ArrayList<>();
  private Map<Double, Rotation2> rotationMap = new TreeMap<>();
  private double length = 0.0;

  private PathSegment.State lastState;

  public SplinePathBuilder(
      Vector2 initialPosition, Rotation2 initialHeading, Rotation2 initialRotation) {
    lastState = new PathSegment.State(initialPosition, initialHeading, 0.0);
    rotationMap.put(0.0, initialRotation);
  }

  private void addSpline(Spline spline) {
    SplinePathSegment segment = new SplinePathSegment(spline);
    segmentList.add(segment);
    lastState = segment.getEnd();
    length += segment.getLength();

    SmartDashboard.putString("Last State", lastState.getPosition().x + " " + lastState.getPosition().y);
    SmartDashboard.putString("First State", segment.getStart().getPosition().x + " " +  segment.getStart().getPosition().y);

  }

  public Path build() {
    return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
  }

  public SplinePathBuilder bezier(Vector2 controlPoint1, Vector2 controlPoint2, Vector2 end) {
    addSpline(new CubicBezierSpline(lastState.getPosition(), controlPoint1, controlPoint2, end));
    return this;
  }

  public SplinePathBuilder bezier(
      Vector2 controlPoint1, Vector2 controlPoint2, Vector2 end, Rotation2 rotation) {
    bezier(controlPoint1, controlPoint2, end);
    rotationMap.put(length, rotation);
    return this;
  }

  public SplinePathBuilder hermite(Vector2 position, Rotation2 heading) {
    addSpline(
        new CubicHermiteSpline(lastState.getPosition(), lastState.getHeading(), position, heading));
    return this;
  }

  public SplinePathBuilder hermite(Vector2 position, Rotation2 heading, Rotation2 rotation) {
    hermite(position, heading);
    rotationMap.put(length, rotation);
    return this;
  }

  public SplinePathBuilder quinticHermite(Vector2 v1, Vector2 position, Vector2 v2) {
    addSpline(
      new QuinticHermiteSpline(lastState.getPosition(), v1, position, v2)
    );
    return this;
  }
}