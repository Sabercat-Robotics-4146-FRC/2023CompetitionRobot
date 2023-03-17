package common.math.spline;

import common.math.Rotation2;
import common.math.Vector2;
import org.ejml.simple.SimpleMatrix;

public class QuinticHermiteSpline extends Spline {
    private static final SimpleMatrix BASIS_MATRIX = 
        new SimpleMatrix(
            new double[][] {
                new double[] {1, 0, 0, 0, 0, 0},
                new double[] {0, 1, 0, 0, 0, 0},
                new double[] {0, 0, 0.5, 0, 0, 0},
                new double[] {-10, -6, -3/2, 10, -4, 1/2},
                new double[] {15, 8, 3/2, -15, 7, -1},
                new double[] {-6, -3, -1/2, 6, -3, 1/2}
            }
        );

    public QuinticHermiteSpline(Vector2 start, Vector2 startTangent, Vector2 end, Vector2 endTangent) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startTangent, end, endTangent, 5));
      }
    
      public QuinticHermiteSpline(
          Vector2 start, Rotation2 startHeading, Vector2 end, Rotation2 endHeading) {
        this(HermiteSplineHelper.createBasisWeightMatrix(start, startHeading, end, endHeading, 5));
      }
    
      private QuinticHermiteSpline(SimpleMatrix basisWeightMatrix) {
        super(BASIS_MATRIX, basisWeightMatrix);
      }
}
