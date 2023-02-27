package common.math.spline;

import common.math.Rotation2;
import common.math.Vector2;
import org.ejml.simple.SimpleMatrix;

public class QuinticHermiteSpline extends Spline {
    // private static final SimpleMatrix BASIS_MATRIX =
    //     new SimpleMatrix(
    //         new double[][] {
    //             new double[] {1, 0, 0, 0, 0, 0},
    //             new double[] {0, 1, 0, 0, 0, 0},
    //             new double[] {0, 0, 1, 0, 0, 0},
    //             new double[] {1, 1, 1, 1, 1, 1},
    //             new double[] {0, 1, 2, 3, 4, 5},
    //             new double[] {0, 0, 2, 6, 12, 20}
    //         }
    //     );

    // private static final SimpleMatrix BASIS_MATRIX = 
    //     new SimpleMatrix(
    //         new double[][] {
    //             new double[] {1,    0,    0,  -10,   15,   -6},
    //             new double[] {0,    1,    0,   -6,    8,   -3},
    //             new double[] {0,    0,  0.5, -1.5,  1.5, -0.5},
    //             new double[] {0,    0,    0,  0.5,   -1,  0.5},
    //             new double[] {0,    0,    0,   -4,    7,   -3},
    //             new double[] {0,    0,    0,   10,  -15,    6}
    //         }
    //     );

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

    // private static final SimpleMatrix BASIS_MATRIX = 
    //     new SimpleMatrix(
    //         new double[][] {
    //             new double[] {-6.0, -3.0, -0.5, 6, -3, 0.5},
    //             new double[] {15.0, 8.0, 1.5, -15.0, 7.0, -1.0},
    //             new double[] {-10.0, -6.0, -1.5, 10.0, -4.0, 0.5},
    //             new double[] {0, 0, 0.5, 0, 0, 0},
    //             new double[] {0, 1.0, 0, 0, 0, 0},
    //             new double[] {1.0, 0, 0, 0, 0, 0}
    //         }
    //     );

    // private static final SimpleMatrix BASIS_MATRIX = 
    //     new SimpleMatrix(
    //           6,
    //           6,
    //           true,
    //           new double[] {
    //             -06.0, -03.0, -00.5, +06.0, -03.0, +00.5, +15.0, +08.0, +01.5, -15.0, +07.0, -01.0,
    //             -10.0, -06.0, -01.5, +10.0, -04.0, +00.5, +00.0, +00.0, +00.5, +00.0, +00.0, +00.0,
    //             +00.0, +01.0, +00.0, +00.0, +00.0, +00.0, +01.0, +00.0, +00.0, +00.0, +00.0, +00.0
    //           });

        // [a₅] = [ -6.0  -3.0  -0.5   6.0  -3.0   0.5][P(i)   ]
      // [a₄] = [ 15.0   8.0   1.5 -15.0   7.0  -1.0][P'(i)  ]
      // [a₃] = [-10.0  -6.0  -1.5  10.0  -4.0   0.5][P"(i)  ]
      // [a₂] = [  0.0   0.0   0.5   0.0   0.0   0.0][P(i+1) ]
      // [a₁] = [  0.0   1.0   0.0   0.0   0.0   0.0][P'(i+1)]
      // [a₀] = [  1.0   0.0   0.0   0.0   0.0   0.0][P"(i+1)]

    //private static final SimpleMatrix INVERSE_BASIS_MATRIX = BASIS_MATRIX.invert();

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
    
    //   public static QuinticHermiteSpline convert(Spline spline) {
    //     if (spline.getDegree() != 5) {
    //       throw new IllegalArgumentException("Spline must be quintic.");
    //     }
    
    //     // B1 * W1 = B2 * W2
    //     // W1 = B1^-1 * B2 * W2
    //     return new QuinticHermiteSpline(
    //         INVERSE_BASIS_MATRIX.mult(spline.getBasisMatrix()).mult(spline.getBasisWeightMatrix()));
    //   }
}
