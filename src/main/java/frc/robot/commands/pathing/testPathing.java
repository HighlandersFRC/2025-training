// // // package frc.robot.commands.pathing;

// // // import edu.wpi.first.wpilibj.Timer;
// // // import edu.wpi.first.wpilibj2.command.Command;
// // // import frc.robot.subsystems.Drive;
// // // import frc.robot.tools.PathLoader.PosePoint;
// // // import frc.robot.tools.math.Vector;
// // // import frc.robot.tools.wrappers.AutoFollower;
// // // import frc.robot.tools.math.PID;
// // // import org.json.JSONObject;
// // // import org.littletonrobotics.junction.Logger;
// // // import java.util.List;

// // // public class testPathing extends AutoFollower {
// // //     private final Drive driveSubsystem;
// // //     private final List<PosePoint> points;
// // //     private final Timer timer = new Timer();
// // //     private final double lookaheadDistance = 0.05;
// // //     private int lastClosestIndex = 0;

// // //     private final PID xPID = new PID(1.3, 0.0, 0.8);
// // //     private final PID yPID = new PID(1.3, 0.0, 0.0);
// // //     private final PID thetaPID = new PID(0.0048, 0.0, 0.011);

// // //     public testPathing(List<PosePoint> points, Drive driveSubsystem) {
// // //         this.points = points;
// // //         this.driveSubsystem = driveSubsystem;
// // //         xPID.setMinOutput(-3.0);
// // //         xPID.setMaxOutput(3.0);
// // //         yPID.setMinOutput(-3.0);
// // //         yPID.setMaxOutput(3.0);
// // //         thetaPID.setMinInput(-180);
// // //         thetaPID.setMaxInput(180);
// // //         thetaPID.setContinuous(true);
// // //         thetaPID.setMinOutput(-0.5);
// // //         thetaPID.setMaxOutput(0.5);
// // //     }

// // //     @Override
// // //     public void initialize() {
// // //         timer.reset();
// // //         timer.start();
// // //         lastClosestIndex = 0;
// // //         double[][] pathPoses = new double[points.size()][3];
// // //         for (int i = 0; i < points.size(); i++) {
// // //             pathPoses[i][0] = points.get(i).x;
// // //             pathPoses[i][1] = points.get(i).y;
// // //             pathPoses[i][2] = points.get(i).theta;
// // //         }
// // //         Logger.recordOutput("Path", pathPoses);
// // //     }

// // //     @Override
// // //     public void execute() {
// // //         double currentTime = timer.get();
// // //         PosePoint robotPose = new PosePoint(
// // //             driveSubsystem.getX(),
// // //             driveSubsystem.getY(),
// // //             driveSubsystem.getAngle(),
// // //             currentTime,
// // //             0.0,
// // //             0.0,
// // //             0.0
// // //         );
// // //         Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});
// // //         int closestIndex = lastClosestIndex;
// // //         double closestDistance = Double.MAX_VALUE;
// // //         for (int i = lastClosestIndex; i < points.size(); i++) {
// // //             double dx = robotPose.x - points.get(i).x;
// // //             double dy = robotPose.y - points.get(i).y;
// // //             double distance = Math.hypot(dx, dy);
// // //             if (distance < closestDistance) {
// // //                 closestDistance = distance;
// // //                 closestIndex = i;
// // //             }
// // //         }
// // //         lastClosestIndex = closestIndex;
// // //         int lookaheadIndex = closestIndex;
// // //         for (int i = closestIndex; i < points.size(); i++) {
// // //             double dx = robotPose.x - points.get(i).x;
// // //             double dy = robotPose.y - points.get(i).y;
// // //             double distance = Math.hypot(dx, dy);
// // //             if (distance > lookaheadDistance) {
// // //                 lookaheadIndex = i;
// // //                 break;
// // //             }
// // //         }
// // //         if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;
// // //         PosePoint target = points.get(lookaheadIndex);
// // //         double vxFF = target.dx;
// // //         double vyFF = target.dy;
// // //         double omegaFF = target.dtheta;
// // //         double xError = target.x - robotPose.x;
// // //         double yError = target.y - robotPose.y;
// // //         xPID.setSetPoint(0);
// // //         yPID.setSetPoint(0);
// // //         thetaPID.setSetPoint(Math.toDegrees(target.theta));
// // //         double xCorrection = xPID.updatePID(-xError);
// // //         double yCorrection = yPID.updatePID(-yError);
// // //         double thetaCorrection = -thetaPID.updatePID(robotPose.theta);
// // //         double vx = vxFF + xCorrection;
// // //         double vy = vyFF + yCorrection;
// // //         double omega = omegaFF + thetaCorrection;
// // //         double headingRad = Math.toRadians(robotPose.theta);
// // //         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
// // //         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);
// // //         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
// // //         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
// // //         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
// // //         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
// // //         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
// // //     }

// // //     @Override
// // //     public void end(boolean interrupted) {
// // //         driveSubsystem.stop();
// // //     }

// // //     @Override
// // //     public boolean isFinished() {
// // //         PosePoint lastPoint = points.get(points.size() - 1);
// // //         double dx = driveSubsystem.getX() - lastPoint.x;
// // //         double dy = driveSubsystem.getY() - lastPoint.y;
// // //         double distanceToEnd = Math.hypot(dx, dy);
// // //         return distanceToEnd < 0.05;
// // //     }

// // //     @Override
// // //     public int getPathPointIndex() {
// // //         return lastClosestIndex;
// // //     }

// // //     @Override
// // //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// // // }
// package frc.robot.commands.pathing;

// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.Drive;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.math.Vector;
// import frc.robot.tools.wrappers.AutoFollower;
// import frc.robot.tools.math.PID;
// import org.json.JSONObject;
// import org.littletonrobotics.junction.Logger;
// import java.util.List;

// public class testPathing extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.5;
//     private int lastClosestIndex = 0;

//     private final PID xPID = new PID(1.3, 0.0, 0.8);
//     private final PID yPID = new PID(1.3, 0.0, 0.0);
//     private final PID thetaPID = new PID(0.0048, 0.0, 0.011);

//     public testPathing(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;
//         xPID.setMinOutput(-3.0);
//         xPID.setMaxOutput(3.0);
//         yPID.setMinOutput(-3.0);
//         yPID.setMaxOutput(3.0);
//         thetaPID.setMinInput(-180);
//         thetaPID.setMaxInput(180);
//         thetaPID.setContinuous(true);
//         thetaPID.setMinOutput(-0.5);
//         thetaPID.setMaxOutput(0.5);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//         lastClosestIndex = 0;
//         double[][] pathPoses = new double[points.size()][3];
//         for (int i = 0; i < points.size(); i++) {
//             pathPoses[i][0] = points.get(i).x;
//             pathPoses[i][1] = points.get(i).y;
//             pathPoses[i][2] = points.get(i).theta;
//         }
//         Logger.recordOutput("Path", pathPoses);
//     }

//     @Override
//     public void execute() {
//         double currentTime = timer.get();
//         PosePoint robotPose = new PosePoint(
//             driveSubsystem.getX(),
//             driveSubsystem.getY(),
//             driveSubsystem.getAngle(),
//             currentTime,
//             0.0,
//             0.0,
//             0.0
//         );

//         Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});

//         int closestIndex = lastClosestIndex;
//         double closestDistance = Double.MAX_VALUE;
//         for (int i = lastClosestIndex; i < points.size(); i++) {
//             double dx = robotPose.x - points.get(i).x;
//             double dy = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dx, dy);
//             if (distance < closestDistance) {
//                 closestDistance = distance;
//                 closestIndex = i;
//             }
//         }
//         lastClosestIndex = closestIndex;

//         int lookaheadIndex = closestIndex;
//         for (int i = closestIndex; i < points.size(); i++) {
//             double dx = robotPose.x - points.get(i).x;
//             double dy = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dx, dy);
//             if (distance > lookaheadDistance) {
//                 lookaheadIndex = i;
//                 break;
//             }
//         }
//         if (lookaheadIndex >= points.size()) {
//             lookaheadIndex = points.size() - 1;
//         }

//         PosePoint target = points.get(lookaheadIndex);

//         double xError = target.x - robotPose.x;
//         double yError = target.y - robotPose.y;

//         xPID.setSetPoint(0);
//         yPID.setSetPoint(0);
//         thetaPID.setSetPoint(Math.toDegrees(target.theta));

//         double xCorrection = xPID.updatePID(-xError);
//         double yCorrection = yPID.updatePID(-yError);
//         double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

//         double vxFF = target.dx*0.0;
//         double vyFF = target.dy*0.0;
//         double omegaFF = Math.toDegrees(target.dtheta);

//         double vx = vxFF + xCorrection;
//         double vy = vyFF + yCorrection;
//         double omega = omegaFF + thetaCorrection;

//         double headingRad = Math.toRadians(robotPose.theta);
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
//         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
//         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
//         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
//         Logger.recordOutput("Velocities", new double [] { target.dx,target.dy,target.dtheta});

//         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//     PosePoint lastPoint = points.get(points.size() - 1);
//     double distx = driveSubsystem.getX() - lastPoint.x;
//     double disty = driveSubsystem.getY() - lastPoint.y;
//     double distanceToEnd = Math.hypot(distx, disty);
    
//     return distanceToEnd < 0.05;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return lastClosestIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// }
package frc.robot.commands.pathing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drive;
import frc.robot.tools.PathLoader.PosePoint;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.tools.math.PID;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class testPathing extends AutoFollower {
    private final Drive driveSubsystem;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.05;
    private int lastClosestIndex = 0;

    private final PID xPID = new PID(1.3, 0.0, 0.8);
    private final PID yPID = new PID(1.3, 0.0, 0.0);
    private final PID thetaPID = new PID(0.004, 0.0, 0.011);

    public testPathing(List<PosePoint> points, Drive driveSubsystem) {
        this.points = points;
        this.driveSubsystem = driveSubsystem;
        xPID.setMinOutput(-3.0);
        xPID.setMaxOutput(3.0);
        yPID.setMinOutput(-3.0);
        yPID.setMaxOutput(3.0);
        thetaPID.setMinInput(-180);
        thetaPID.setMaxInput(180);
        thetaPID.setContinuous(true);
        thetaPID.setMinOutput(-0.5);
        thetaPID.setMaxOutput(0.5);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        lastClosestIndex = 0;
        PosePoint start = points.get(0);
        driveSubsystem.resetOdometry(new Pose2d(start.x, start.y, new Rotation2d(start.theta)));
        double[][] pathPoses = new double[points.size()][3];
        for (int i = 0; i < points.size(); i++) {
            pathPoses[i][0] = points.get(i).x;
            pathPoses[i][1] = points.get(i).y;
            pathPoses[i][2] = points.get(i).theta;
        }
        Logger.recordOutput("Path", pathPoses);
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PosePoint robotPose = new PosePoint(
            driveSubsystem.getX(),
            driveSubsystem.getY(),
            driveSubsystem.getAngle(),
            currentTime,
            0.0,
            0.0,
            0.0
        );

        Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});

        int closestIndex = lastClosestIndex;
        double closestDistance = Double.MAX_VALUE;
        for (int i = lastClosestIndex; i < points.size(); i++) {
            double dx = robotPose.x - points.get(i).x;
            double dy = robotPose.y - points.get(i).y;
            double distance = Math.hypot(dx, dy);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = i;
            }
        }
        lastClosestIndex = closestIndex;

        int lookaheadIndex = closestIndex;
        for (int i = closestIndex; i < points.size(); i++) {
            double dx = robotPose.x - points.get(i).x;
            double dy = robotPose.y - points.get(i).y;
            double distance = Math.hypot(dx, dy);
            if (distance > lookaheadDistance) {
                lookaheadIndex = i;
                break;
            }
        }
        if (lookaheadIndex >= points.size()) {
            lookaheadIndex = points.size() - 1;
        }

        PosePoint target = points.get(lookaheadIndex);

        double xError = target.x - robotPose.x;
        double yError = target.y - robotPose.y;

        xPID.setSetPoint(0);
        yPID.setSetPoint(0);
        thetaPID.setSetPoint(Math.toDegrees(target.theta));

        double xCorrection = xPID.updatePID(-xError);
        double yCorrection = yPID.updatePID(-yError);
        double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

        double vxFF = target.dx*0.5;
        double vyFF = target.dy*0.5;
        double omegaFF = Math.toDegrees(target.dtheta);

        double vx = vxFF + xCorrection;
        double vy = vyFF + yCorrection;
        double omega = omegaFF + thetaCorrection;

        double headingRad = Math.toRadians(robotPose.theta);
        double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

        Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
        Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
        Logger.recordOutput("Velocities", new double [] { target.dx,target.dy,target.dtheta});

        driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        PosePoint lastPoint = points.get(points.size() - 1);
        double distx = driveSubsystem.getX() - lastPoint.x;
        double disty = driveSubsystem.getY() - lastPoint.y;
        double distanceToEnd = Math.hypot(distx, disty);
        return distanceToEnd < 0.05;
    }

    @Override
    public int getPathPointIndex() {
        return lastClosestIndex;
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
}
// package frc.robot.commands.pathing;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.subsystems.Drive;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.math.Vector;
// import frc.robot.tools.wrappers.AutoFollower;
// import frc.robot.tools.math.PID;
// import org.json.JSONObject;
// import org.littletonrobotics.junction.Logger;
// import java.util.List;

// public class testPathing extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.5;
//     private int lastClosestIndex = 0;

//     private final PID xPID = new PID(1.3, 0.0, 0.8);
//     private final PID yPID = new PID(1.3, 0.0, 0.0);
//     private final PID thetaPID = new PID(0.004, 0.0, 0.011);

//     public testPathing(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;
//         xPID.setMinOutput(-3.0);
//         xPID.setMaxOutput(3.0);
//         yPID.setMinOutput(-3.0);
//         yPID.setMaxOutput(3.0);
//         thetaPID.setMinInput(-180);
//         thetaPID.setMaxInput(180);
//         thetaPID.setContinuous(true);
//         thetaPID.setMinOutput(-0.5);
//         thetaPID.setMaxOutput(0.5);
//     }


//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//         lastClosestIndex = 0;
//         PosePoint start = points.get(0);
//         driveSubsystem.resetOdometry(new Pose2d(start.x, start.y, new Rotation2d(start.theta)));
//         double[][] pathPoses = new double[points.size()][3];
//         for (int i = 0; i < points.size(); i++) {
//             pathPoses[i][0] = points.get(i).x;
//             pathPoses[i][1] = points.get(i).y;
//             pathPoses[i][2] = points.get(i).theta;
//         }
//         Logger.recordOutput("Path", pathPoses);
//     }

//     @Override
//     public void execute() {
//         double currentTime = timer.get();
//         PosePoint robotPose = new PosePoint(
//             driveSubsystem.getX(),
//             driveSubsystem.getY(),
//             driveSubsystem.getAngle(),
//             currentTime,
//             0.0,
//             0.0,
//             0.0
//         );

//         Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});

//         int closestIndex = lastClosestIndex;
//         double closestDistance = Double.MAX_VALUE;
//         for (int i = lastClosestIndex; i < points.size(); i++) {
//             double dxp = robotPose.x - points.get(i).x;
//             double dyp = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dxp, dyp);
//             if (distance < closestDistance) {
//                 closestDistance = distance;
//                 closestIndex = i;
//             }
//         }
//         lastClosestIndex = closestIndex;

//         int lookaheadIndex = closestIndex;
//         for (int i = closestIndex; i < points.size(); i++) {
//             double dxp = robotPose.x - points.get(i).x;
//             double dyp = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dxp, dyp);
//             if (distance > lookaheadDistance) {
//                 lookaheadIndex = i;
//                 break;
//             }
//         }
//         if (lookaheadIndex >= points.size()) {
//             lookaheadIndex = points.size() - 1;
//         }

//         PosePoint target = points.get(lookaheadIndex);

//         double xError = target.x - robotPose.x;
//         double yError = target.y - robotPose.y;

//         xPID.setSetPoint(0);
//         yPID.setSetPoint(0);
//         thetaPID.setSetPoint(Math.toDegrees(target.theta));

//         double xCorrection = xPID.updatePID(-xError);
//         double yCorrection = yPID.updatePID(-yError);
//         double thetaCorrectionDeg = -thetaPID.updatePID(robotPose.theta);

//         double vxFF = target.dx;
//         double vyFF = target.dy;
//         double omegaFF = target.dtheta;

//         if (Math.abs(vxFF) + Math.abs(vyFF) < 1e-9) {
//             int a = lookaheadIndex > 0 ? lookaheadIndex - 1 : lookaheadIndex;
//             int b = lookaheadIndex < points.size() - 1 ? lookaheadIndex + 1 : lookaheadIndex;
//             if (a != lookaheadIndex) {
//                 PosePoint prev = points.get(a);
//                 double dt = target.time - prev.time;
//                 if (dt > 1e-6) {
//                     vxFF = (target.x - prev.x) / dt;
//                     vyFF = (target.y - prev.y) / dt;
//                 }
//             } else if (b != lookaheadIndex) {
//                 PosePoint next = points.get(b);
//                 double dt = next.time - target.time;
//                 if (dt > 1e-6) {
//                     vxFF = (next.x - target.x) / dt;
//                     vyFF = (next.y - target.y) / dt;
//                 }
//             }
//         }

//         if (Math.abs(omegaFF) < 1e-9) {
//             int a = lookaheadIndex > 0 ? lookaheadIndex - 1 : lookaheadIndex;
//             int b = lookaheadIndex < points.size() - 1 ? lookaheadIndex + 1 : lookaheadIndex;
//             if (a != lookaheadIndex) {
//                 PosePoint prev = points.get(a);
//                 double dt = target.time - prev.time;
//                 if (dt > 1e-6) {
//                     double dth = Math.atan2(Math.sin(target.theta - prev.theta), Math.cos(target.theta - prev.theta));
//                     omegaFF = dth / dt;
//                 }
//             } else if (b != lookaheadIndex) {
//                 PosePoint next = points.get(b);
//                 double dt = next.time - target.time;
//                 if (dt > 1e-6) {
//                     double dth = Math.atan2(Math.sin(next.theta - target.theta), Math.cos(next.theta - target.theta));
//                     omegaFF = dth / dt;
//                 }
//             }
//         }

//         double vx = vxFF + xCorrection;
//         double vy = vyFF + yCorrection;
//         double omega = omegaFF + Math.toRadians(thetaCorrectionDeg);

//         double headingRad = Math.toRadians(robotPose.theta);
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
//         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrectionDeg});
//         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
//         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
//         Logger.recordOutput("Velocities", new double [] { target.dx, target.dy, target.dtheta });

//         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         PosePoint lastPoint = points.get(points.size() - 1);
//         double distx = driveSubsystem.getX() - lastPoint.x;
//         double disty = driveSubsystem.getY() - lastPoint.y;
//         double distanceToEnd = Math.hypot(distx, disty);
//         return distanceToEnd < 0.05;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return lastClosestIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// }
