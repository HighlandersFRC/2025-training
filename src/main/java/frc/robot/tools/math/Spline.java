package frc.robot.tools.math;

import java.util.Arrays;

public final class Spline {
    QuiticHermiteSpline x, y, theta;
    Waypoint[] waypoints;
    Derivatives[] xderivatives, yderivatives, thetaderivatives;

    /**
     * Constructs a spline from the given waypoints.
     *
     * @param waypoints An array of Waypoint objects representing the path.
     */
    public Spline(Waypoint[] waypoints) {
        Arrays.sort(waypoints, (w1, w2) -> Double.compare(w1.t, w2.t));
        this.waypoints = waypoints;
        optimizeRotation();
        this.xderivatives = new Derivatives[waypoints.length];
        this.yderivatives = new Derivatives[waypoints.length];
        this.thetaderivatives = new Derivatives[waypoints.length];

        for (int i = 0; i < waypoints.length; i++) {
            Waypoint w = waypoints[i];
            this.xderivatives[i] = new Derivatives(w.t, w.x, w.dx, w.d2x);
            this.yderivatives[i] = new Derivatives(w.t, w.y, w.dy, w.d2y);
            this.thetaderivatives[i] = new Derivatives(w.t, w.theta, w.dtheta, w.d2theta);
        }

        this.x = new QuiticHermiteSpline(this.xderivatives);
        this.y = new QuiticHermiteSpline(this.yderivatives);
        this.theta = new QuiticHermiteSpline(this.thetaderivatives);
    }

    /**
     * Optimizes the rotation of the waypoints to ensure smooth transitions.
     * This method adjusts the theta values of the waypoints to minimize abrupt
     * changes in direction.
     */
    void optimizeRotation() {
        for (int i = 1; i < waypoints.length; i++) {
            var p1 = waypoints[i - 1];
            waypoints[i].theta %= (2 * Math.PI);
            while (true) {
                if (waypoints[i].theta - p1.theta > Math.PI) {
                    waypoints[i].theta -= 2 * Math.PI;
                } else if (waypoints[i].theta - p1.theta < -Math.PI) {
                    waypoints[i].theta += 2 * Math.PI;
                } else {
                    break;
                }
            }
        }
    }

    /**
     * Interpolates the waypoint at a given time using the spline.
     *
     * @param time The time at which to interpolate the waypoint.
     * @return The interpolated Waypoint at the specified time.
     * @throws IndexOutOfBoundsException if the time is out of bounds for the spline
     *                                   segments.
     */
    public Waypoint interpolate(double time) {
        if (time < waypoints[0].t || time > waypoints[waypoints.length - 1].t) {
            throw new IndexOutOfBoundsException("Time out of bounds for spline segments");
        }

        Derivatives DerX = this.x.interpolate(time);
        Derivatives DerY = this.y.interpolate(time);
        Derivatives DerTheta = this.theta.interpolate(time);

        return new Waypoint(time, DerX.d0, DerY.d0, DerTheta.d0, DerX.d1, DerY.d1, DerTheta.d1, DerX.d2, DerY.d2,
                DerTheta.d2);
    }

    /**
     * Calculates the curvature of the spline at a given time.
     *
     * @param time The time at which to calculate the curvature.
     * @return The curvature at the specified time.
     * @throws IndexOutOfBoundsException if the time is out of bounds for the spline
     *                                   segments.
     */
    public double curvature(double time) {
        if (time < waypoints[0].t || time > waypoints[waypoints.length - 1].t) {
            throw new IndexOutOfBoundsException("Time out of bounds for spline segments");
        }

        Derivatives DerX = this.x.interpolate(time);
        Derivatives DerY = this.y.interpolate(time);

        double dx = DerX.d1;
        double dy = DerY.d1;
        double d2x = DerX.d2;
        double d2y = DerY.d2;

        return Math.abs(dx * d2y - dy * d2x) / Math.pow(dx * dx + dy * dy, 1.5);
    }
}
