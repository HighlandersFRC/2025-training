package frc.robot.tools.utils;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.tools.math.Spline;
import frc.robot.tools.math.Waypoint;

public class PolarPathing {
    public static Pose2d jsonToPose2d(JSONObject point) {
        double x = point.getDouble("x");
        double y = point.getDouble("y");
        double heading = point.getDouble("angle");
        return new Pose2d(x, y, new Rotation2d(heading));
    }

    public static Translation2d jsonToTranslation2d(JSONObject point) {
        double x = point.getDouble("x");
        double y = point.getDouble("y");
        return new Translation2d(x, y);
    }

    public static ChassisSpeeds jsonToVelocities(JSONObject point) {
        double vx = point.getDouble("x_velocity");
        double vy = point.getDouble("y_velocity");
        double omega = point.getDouble("angular_velocity");
        return new ChassisSpeeds(vx, vy, omega);
    }

    public static ChassisSpeeds jsonToAccelerations(JSONObject point) {
        double ax = point.getDouble("x_acceleration");
        double ay = point.getDouble("y_acceleration");
        double alpha = point.getDouble("angular_acceleration");
        return new ChassisSpeeds(ax, ay, alpha);
    }

    public static Spline pathJsonToSpline(JSONObject path) {
        Waypoint[] a = new Waypoint[path.getJSONArray("key_points").length()];
        java.util.concurrent.atomic.AtomicInteger i = new java.util.concurrent.atomic.AtomicInteger(0);
        path.getJSONArray("key_points").forEach(point -> {
            a[i.get()] = (jsonToWaypoint((JSONObject) point));
            i.incrementAndGet();
        });
        return new Spline(a);
    }

    public static Waypoint jsonToWaypoint(JSONObject point) {
        double x = point.getDouble("x");
        double y = point.getDouble("y");
        double theta = point.getDouble("angle");
        double t = point.getDouble("time");
        double dx = point.optDouble("x_velocity", 0);
        double dy = point.optDouble("y_velocity", 0);
        double dtheta = point.optDouble("angular_velocity", 0);
        double d2x = point.optDouble("x_acceleration", 0);
        double d2y = point.optDouble("y_acceleration", 0);
        double d2theta = point.optDouble("angular_acceleration", 0);
        return new Waypoint(t, x, y, theta, dx, dy, dtheta, d2x, d2y, d2theta);
    }
}
