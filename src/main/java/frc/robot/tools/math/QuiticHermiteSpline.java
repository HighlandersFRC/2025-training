package frc.robot.tools.math;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

public class QuiticHermiteSpline {
    final Matrix<N6, N6> scale = new Matrix<N6, N6>(new SimpleMatrix(6, 6, true, new double[] {
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 0.5, 0, 0, 0,
            -10, -6, -1.5, 10, -4, 0.5,
            15, 8, 1.5, -15, 7, -1,
            -6, -3, -0.5, 6, -3, 0.5
    }));
    final int numSegments;
    final Matrix<N6, N1>[] segmentCoefficients;
    final Derivatives[] derivatives;

    /**
     * Constructs a quintic Hermite spline from the given derivatives.
     *
     * @param derivatives An array of Derivatives objects representing the
     *                    derivatives at each waypoint.
     */
    @SuppressWarnings("unchecked")
    public QuiticHermiteSpline(Derivatives[] derivatives) {
        this.derivatives = derivatives;
        numSegments = derivatives.length - 1;
        segmentCoefficients = new Matrix[numSegments];
        for (int i = 0; i < numSegments; i++) {
            segmentCoefficients[i] = new Matrix<N6, N1>(new SimpleMatrix(6, 1, true, new double[] {
                    derivatives[i].d0,
                    derivatives[i].d1,
                    derivatives[i].d2,
                    derivatives[i + 1].d0,
                    derivatives[i + 1].d1,
                    derivatives[i + 1].d2
            }));
            segmentCoefficients[i] = scale.times(segmentCoefficients[i]);
        }
    }

    /**
     * Interpolates the derivatives at a given time using the quintic Hermite
     * spline.
     *
     * @param time The time at which to interpolate the derivatives.
     * @return The derivatives at the specified time.
     * @throws IndexOutOfBoundsException if the time is out of bounds for the spline
     *                                   segments.
     */
    Derivatives interpolate(double time) {
        // Check if the time is within the bounds of the spline segments
        if (time < derivatives[0].t || time > derivatives[numSegments].t) {
            throw new IndexOutOfBoundsException("Time out of bounds for spline segments");
        }

        // Find the segment index for the given time
        int segmentIndex = -1;
        for (int i = 0; i < numSegments; i++) {
            if (time <= derivatives[i + 1].t) {
                System.out.println(derivatives[i + 1].t);
                segmentIndex = i;
                break;
            }
        }

        // Calculate the time relative to the start of the segment between 0 and 1
        double t = time - derivatives[segmentIndex].t;
        double deltaT = derivatives[segmentIndex + 1].t - derivatives[segmentIndex].t;
        double tau = t / deltaT;
        Matrix<N6, N1> coefficients = segmentCoefficients[segmentIndex];

        // Calculate the powers of tau
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        // Calculate the derivatives at the given time
        double d0 = coefficients.get(0, 0) + coefficients.get(1, 0) * tau + coefficients.get(2, 0) * tau2
                + coefficients.get(3, 0) * tau3 + coefficients.get(4, 0) * tau4 + coefficients.get(5, 0) * tau5;
        double d1 = coefficients.get(1, 0) + 2 * coefficients.get(2, 0) * tau + 3 * coefficients.get(3, 0) * tau2
                + 4 * coefficients.get(4, 0) * tau3 + 5 * coefficients.get(5, 0) * tau4;
        double d2 = 2 * coefficients.get(2, 0) + 6 * coefficients.get(3, 0) * tau
                + 12 * coefficients.get(4, 0) * tau2 + 20 * coefficients.get(5, 0) * tau3;

        return new Derivatives(time, d0, d1, d2);
    }
}