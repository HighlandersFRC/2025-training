 // // package frc.robot.commands.Pathing;

// // // import frc.robot.subsystems.Drive;
// // // import frc.robot.tools.math.Vector;
// // // import frc.robot.tools.math.PID;
// // // import frc.robot.tools.PathLoader.PosePoint;
// // // import frc.robot.tools.wrappers.AutoFollower;

// // // import java.util.List;

// // // import org.json.JSONObject;

// // // import edu.wpi.first.wpilibj.Timer;
// // // import edu.wpi.first.math.geometry.Pose2d;

// // // public class FollowTest extends AutoFollower {
// // //     private final Drive drive;
// // //     private final double lookaheadDistance = 0.3;

// // //     private final List<PosePoint> points;
// // //     private int currentIndex;
// // //     private final PID xPID;
// // //     private final PID yPID;
// // //     private final PID thetaPID;

// // //     private final Timer timer = new Timer();
// // //     private final double pathEndTime;

// // //     public FollowTest(List<PosePoint> points, Drive drive) {
// // //         this.points = points;
// // //         this.drive = drive;

// // //         xPID = new PID(4, 0.0, 2.1);
// // //         yPID = new PID(4, 0.0, 2.1);
// // //         thetaPID = new PID(2, 0.0, 2.9);
// // //         xPID.setMinOutput(-1.0);
// // //         xPID.setMaxOutput(1.0);
// // //         yPID.setMinOutput(-1.0);
// // //         yPID.setMaxOutput(1.0);
      

// // //         thetaPID.setContinuous(true);
// // //         thetaPID.setMinInput(-180);
// // //         thetaPID.setMaxInput(180);
        
// // //         currentIndex = 0;

// // //         if (points.isEmpty()) {
// // //             pathEndTime = 0;
// // //         } else {
// // //             pathEndTime = points.get(points.size() - 1).time;
// // //         }
// // //     }

// // //     @Override
// // //     public void initialize() {
// // //         timer.reset();
// // //         timer.start();
// // //         currentIndex = 0;
// // //     }

// // //     @Override
// // //     public void execute() {
// // //         if (points.isEmpty()) {
// // //             drive.stop();
// // //             return;
// // //         }

// // //         Pose2d currentPose = drive.getPose2D();
// // //         double timeNow = timer.get();

// // //         while (currentIndex < points.size() - 1 && points.get(currentIndex).time < timeNow) {
// // //             currentIndex++;
// // //         }

// // //         PosePoint lookahead = points.get(currentIndex);
// // //         for (int i = currentIndex; i < points.size(); i++) {
// // //             PosePoint p = points.get(i);
// // //             double dx = p.x - currentPose.getX();
// // //             double dy = p.y - currentPose.getY();
// // //             if (Math.hypot(dx, dy) >= lookaheadDistance) {
// // //                 lookahead = p;
// // //                 break;
// // //             }
// // //         }

// // //         xPID.setSetPoint(lookahead.x);
// // //         yPID.setSetPoint(lookahead.y);
// // //         thetaPID.setSetPoint(Math.toDegrees(lookahead.theta));

// // //         double xSpeed = xPID.updatePID(currentPose.getX());
// // //         double ySpeed = yPID.updatePID(currentPose.getY());
       

// // //         Vector fieldVector = new Vector(xSpeed, ySpeed);

// // //         drive.autoDrive(fieldVector, Math.toDegrees(lookahead.theta));
// // //     }

// // //     @Override
// // //     public void end(boolean interrupted) {
// // //         drive.stop();
// // //     }

// // //     @Override
// // //     public boolean isFinished() {
// // //         return currentIndex >= points.size() - 1 && timer.get() > pathEndTime;
// // //     }

// // //     @Override
// // //     public int getPathPointIndex() {
// // //         return currentIndex;
// // //     }

// // //     @Override
// // //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
// // //         // TODO Auto-generated method stub
// // //         throw new UnsupportedOperationException("Unimplemented method 'from'");
// // //     }
// // // }
// // package frc.robot.commands.Pathing;

// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.Drive;
// // import frc.robot.tools.PathLoader.PosePoint;
// // import frc.robot.tools.math.Vector;
// // import frc.robot.tools.wrappers.AutoFollower;
// // import frc.robot.tools.math.PID;
// // import org.json.JSONObject;
// // import org.littletonrobotics.junction.Logger;
// // import java.util.List;

// // public class FollowTest extends AutoFollower {
// //     private final Drive driveSubsystem;
// //     private final List<PosePoint> points;
// //     private final Timer timer = new Timer();
// //     private final double lookaheadDistance = 0.3;
// //     private int lastClosestIndex = 0;

// //     private final PID xPID = new PID(1.65, 0.0, 4.3);
// //     private final PID yPID = new PID(1.65, 0.0, 4.3);
// //     private final PID thetaPID = new PID(0.0044, 0.0,0.011);

// //     public FollowTest(List<PosePoint> points, Drive driveSubsystem) {
// //         this.points = points;
// //         this.driveSubsystem = driveSubsystem;

// //         xPID.setMinOutput(-3.0);
// //         xPID.setMaxOutput(3.0);

// //         yPID.setMinOutput(-3.0);
// //         yPID.setMaxOutput(3.0);

// //         thetaPID.setMinInput(-180);
// //         thetaPID.setMaxInput(180);
// //         thetaPID.setContinuous(false);
// //         thetaPID.setMinOutput(-1);
// //         thetaPID.setMaxOutput(1);
// //     }

// //     @Override
// //     public void initialize() {
// //         timer.reset();
// //         timer.start();
// //         lastClosestIndex = 0;

// //         double[][] pathPoses = new double[points.size()][3];
// //         for (int i = 0; i < points.size(); i++) {
// //             pathPoses[i][0] = points.get(i).x;
// //             pathPoses[i][1] = points.get(i).y;
// //             pathPoses[i][2] = points.get(i).theta;
// //         }

// //         Logger.recordOutput("Path", pathPoses);
// //     }

// //     @Override
// //     public void execute() {
// //         double currentTime = timer.get();

// //         PosePoint robotPose = new PosePoint(
// //             driveSubsystem.getX(),
// //             driveSubsystem.getY(),
// //             driveSubsystem.getAngle(),
// //             currentTime
// //         );

// //         Logger.recordOutput("Field/Robot", new double[] {
// //             robotPose.x, robotPose.y, robotPose.theta
// //         });

// //         int closestIndex = lastClosestIndex;
// //         double closestDistance = Double.MAX_VALUE;
// //         for (int i = lastClosestIndex; i < points.size(); i++) {
// //             double dx = robotPose.x - points.get(i).x;
// //             double dy = robotPose.y - points.get(i).y;
// //             double distance = Math.hypot(dx, dy);
// //             if (distance < closestDistance) {
// //                 closestDistance = distance;
// //                 closestIndex = i;
// //             }
// //         }
// //         lastClosestIndex = closestIndex;

// //         int lookaheadIndex = closestIndex;
// //         for (int i = closestIndex; i < points.size(); i++) {
// //             double dx = robotPose.x - points.get(i).x;
// //             double dy = robotPose.y - points.get(i).y;
// //             double distance = Math.hypot(dx, dy);
// //             if (distance > lookaheadDistance) {
// //                 lookaheadIndex = i;
// //                 break;
// //             }
// //         }

// //         if (lookaheadIndex >= points.size()) {
// //             lookaheadIndex = points.size() - 1;
// //         }

// //         PosePoint target = points.get(lookaheadIndex);
// //         PosePoint prev = points.get(Math.max(lookaheadIndex - 1, 0));

// //         double dt = target.time - prev.time;
// //         if (dt <= 0) dt = 0.02;

// //         double vxFF = (target.x - prev.x) / dt;
// //         double vyFF = (target.y - prev.y) / dt;
// //         double vThetaFF = (target.theta - prev.theta) / dt;

// //         double xError = target.x - robotPose.x;
// //         double yError = target.y - robotPose.y;
// //         double thetaErrorDeg = Math.toDegrees(target.theta) - robotPose.theta;

// //         xPID.setSetPoint(0);
// //         yPID.setSetPoint(0);
// //         thetaPID.setSetPoint(0);

// //         double xCorrection = xPID.updatePID(-xError);
// //         double yCorrection = yPID.updatePID(-yError);
// //         double thetaCorrection = thetaPID.updatePID(-thetaErrorDeg);

// //         double vx = vxFF + xCorrection;
// //         double vy = vyFF + yCorrection;
// //         double omega = vThetaFF + thetaCorrection;

// //         double headingRad = Math.toRadians(robotPose.theta);
// //         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
// //         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

// //         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
// //         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
// //         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, vThetaFF});
// //         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});

// //         driveSubsystem.autoDrive(new Vector(robotX, robotY), omega);
// //     }

// //     @Override
// //     public void end(boolean interrupted) {
// //         driveSubsystem.stop();
// //     }

// //     @Override
// //     public boolean isFinished() {
// //         return lastClosestIndex >= points.size() - 1;
// //     }

// //     @Override
// //     public int getPathPointIndex() {
// //         return lastClosestIndex;
// //     }

// //     @Override
// //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// // }
// package frc.robot.commands.Pathing;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drive;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.math.Vector;
// import frc.robot.tools.wrappers.AutoFollower;
// import frc.robot.tools.math.PID;
// import org.json.JSONObject;
// import org.littletonrobotics.junction.Logger;
// import java.util.List;

// public class FollowTest extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.3;
//     private int lastClosestIndex = 0;

//     private final PID xPID = new PID(1.65, 0.0, 4.3);
//     private final PID yPID = new PID(1.65, 0.0, 4.3);
//     private final PID thetaPID = new PID(0.0044, 0.0, 0.011);

//     public FollowTest(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;

//         xPID.setMinOutput(-3.0);
//         xPID.setMaxOutput(3.0);

//         yPID.setMinOutput(-3.0);
//         yPID.setMaxOutput(3.0);

//         thetaPID.setMinInput(-180);
//         thetaPID.setMaxInput(180);
//         thetaPID.setContinuous(false);
//         thetaPID.setMinOutput(-1);
//         thetaPID.setMaxOutput(1);
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
//             currentTime
//         );

//         Logger.recordOutput("Field/Robot", new double[] {
//             robotPose.x, robotPose.y, robotPose.theta
//         });

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
//         PosePoint prev = points.get(Math.max(lookaheadIndex - 1, 0));

//         double dt = target.time - prev.time;
//         if (dt <= 0) dt = 0.02;

//         double vxFF = (target.x - prev.x) / dt;
//         double vyFF = (target.y - prev.y) / dt;
//         double vThetaFF = (target.theta - prev.theta) / dt;

//         double xError = target.x - robotPose.x;
//         double yError = target.y - robotPose.y;
//         double thetaErrorDeg = Math.toDegrees(target.theta) - robotPose.theta;

//         xPID.setSetPoint(0);
//         yPID.setSetPoint(0);
//         thetaPID.setSetPoint(0);

//         double xCorrection = xPID.updatePID(-xError);
//         double yCorrection = yPID.updatePID(-yError);
//         double thetaCorrection = thetaPID.updatePID(-thetaErrorDeg);

//         double vx = vxFF + xCorrection;
//         double vy = vyFF + yCorrection;
//         double omega = Math.toDegrees(vThetaFF) + thetaCorrection;

//         double headingRad = Math.toRadians(robotPose.theta);
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
//         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
//         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, vThetaFF});
//         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});

//         driveSubsystem.autoDrive(new Vector(robotX, robotY), omega);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return lastClosestIndex >= points.size() - 1;
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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.tools.PathLoaderOld.PosePoint;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.tools.math.PID;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class FollowTest extends AutoFollower {
    private final Drive driveSubsystem;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.5;
    private int lastClosestIndex = 0;

    private final PID xPID = new PID(1.3, 0.0, 0.8);
    private final PID yPID = new PID(1.3, 0.0, 0.);
    private final PID thetaPID = new PID(0.0048, 0.0, 0.011);

    public FollowTest(List<PosePoint> points, Drive driveSubsystem) {
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
            currentTime
        );

        Logger.recordOutput("Field/Robot", new double[] {
            robotPose.x, robotPose.y, robotPose.theta
        });

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
        PosePoint prev = points.get(Math.max(lookaheadIndex - 1, 0));

        double dt = target.time - prev.time;
        // if (dt <= 0) dt = 0.02;
        double vxFF = ((target.x - prev.x) / dt)*0.0;
        double vyFF = ((target.y - prev.y) / dt)*0.0;

        double xError = target.x - robotPose.x;
        double yError = target.y - robotPose.y;

        xPID.setSetPoint(0);
        yPID.setSetPoint(0);
        thetaPID.setSetPoint(Math.toDegrees(target.theta));

        double xCorrection = xPID.updatePID(-xError);
        double yCorrection = yPID.updatePID(-yError);
        double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

        double vx = vxFF + xCorrection;
        double vy = vyFF + yCorrection;
        double omega = thetaCorrection;

        double headingRad = Math.toRadians(robotPose.theta);
        double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);
    
        Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, 0});
        Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});

        driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return lastClosestIndex >= points.size() - 1;
    }

    @Override
    public int getPathPointIndex() {
        return lastClosestIndex;
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
}
