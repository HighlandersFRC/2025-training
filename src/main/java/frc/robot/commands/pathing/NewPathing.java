// // // package frc.robot.commands.pathing;

// // // import edu.wpi.first.wpilibj.Timer;
// // // import edu.wpi.first.wpilibj2.command.CommandScheduler;
// // // import edu.wpi.first.math.geometry.Pose2d;
// // // import edu.wpi.first.math.geometry.Rotation2d;
// // // import frc.robot.subsystems.Drive;
// // // import frc.robot.tools.PathLoader.PosePoint;
// // // import frc.robot.tools.math.Vector;
// // // import frc.robot.tools.wrappers.AutoFollower;
// // // import frc.robot.tools.math.PID;
// // // import org.json.JSONObject;
// // // import org.littletonrobotics.junction.Logger;
// // // import java.util.List;

// // // public class NewPathing extends AutoFollower {
// // //     private final Drive driveSubsystem;
// // //     private final List<PosePoint> points;
// // //     private final Timer timer = new Timer();
// // //     private final double lookaheadDistance = 0.1;
// // //     private int lastLookaheadIndex = 0;

// // //     private final PID xPID = new PID(0.35, 0.0, 0.73);
// // //     private final PID yPID = new PID(0.35, 0.0, 0.73);
// // //     private final PID thetaPID = new PID(0.005, 0.0, 0.0);  

// // //     public NewPathing(List<PosePoint> points, Drive driveSubsystem) {
// // //         this.points = points;
// // //         this.driveSubsystem = driveSubsystem;
// // //         xPID.setMinOutput(-3.0); xPID.setMaxOutput(3.0);
// // //         yPID.setMinOutput(-3.0); yPID.setMaxOutput(3.0);
// // //         thetaPID.setMinInput(-Math.PI); thetaPID.setMaxInput(Math.PI); thetaPID.setContinuous(true);
// // //         thetaPID.setMinOutput(-0.5); thetaPID.setMaxOutput(0.5);
// // //     }

// // //     @Override
// // //     public void initialize() {
// // //         timer.reset();
// // //         timer.start();
// // //         lastLookaheadIndex = 0;
// // //         PosePoint start = points.get(0);
// // //         driveSubsystem.resetOdometry(new Pose2d(start.x, start.y, new Rotation2d(start.theta)));
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
// // //             0.0, 0.0, 0.0
// // //         );

// // //         Logger.recordOutput("Field/Robot", new double[] {
// // //             robotPose.x, robotPose.y, robotPose.theta
// // //         });

// // //         int lookaheadIndex = lastLookaheadIndex;
// // //         for (int i = lastLookaheadIndex; i < points.size(); i++) {
// // //             double dx = robotPose.x - points.get(i).x;
// // //             double dy = robotPose.y - points.get(i).y;
// // //             double distance = Math.hypot(dx, dy);
// // //             if (distance > lookaheadDistance) {
// // //                 lookaheadIndex = i;
// // //                 break;
// // //             }
// // //         }

// // //         if (lookaheadIndex >= points.size()) {
// // //             lookaheadIndex = points.size() - 1;
// // //         }
// // //         if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) {
// // //             lookaheadIndex++;
// // //         }

// // //         PosePoint target = points.get(lookaheadIndex);
// // //         lastLookaheadIndex = lookaheadIndex;

// // //         double xError = target.x - robotPose.x;
// // //         double yError = target.y - robotPose.y;

// // //         xPID.setSetPoint(0);
// // //         yPID.setSetPoint(0);
// // //         thetaPID.setSetPoint(target.theta);

// // //         double xCorrection = xPID.updatePID(-xError);
// // //         double yCorrection = yPID.updatePID(-yError);
// // //         double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

// // //         double vxFF = target.dx * 0.1;
// // //         double vyFF = target.dy * 0.1;

// // //         double vx = vxFF + xCorrection;
// // //         double vy = vyFF + yCorrection;
// // //         double omega = thetaCorrection;

// // //         double headingRad = robotPose.theta;
// // //         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
// // //         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

// // //         Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
// // //         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
// // //         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
// // //         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
// // //         Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

// // //         driveSubsystem.driveSwerve(new Vector(robotX, robotY), Math.toDegrees(omega));
// // //     }

// // //     @Override
// // //     public void end(boolean interrupted) {
// // //         driveSubsystem.stop();
// // //     }

// // //     @Override
// // //     public boolean isFinished() {
// // //         PosePoint lastPoint = points.get(points.size() - 1);
// // //         double distx = driveSubsystem.getX() - lastPoint.x;
// // //         double disty = driveSubsystem.getY() - lastPoint.y;
// // //         double distanceToEnd = Math.hypot(distx, disty);
// // //         return distanceToEnd < 0.05 && lastLookaheadIndex >= points.size() - 1;
// // //     }

// // //     @Override
// // //     public int getPathPointIndex() {
// // //         return lastLookaheadIndex;
// // //     }

// // //     @Override
// // //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
// // //         points.clear();
// // //         for (int i = pointIndex; i <= toIndex; i++) {
// // //             JSONObject point = pathJSON.getJSONObject(String.valueOf(i));
// // //             double x = point.getDouble("x");
// // //             double y = point.getDouble("y");
// // //             double theta = point.getDouble("theta");
// // //             double dx = point.optDouble("dx", 0.0);
// // //             double dy = point.optDouble("dy", 0.0);
// // //             double dtheta = point.optDouble("dtheta", 0.0);
// // //             points.add(new PosePoint(x, y, theta, 0.0, dx, dy, dtheta));
// // //         }
// // //     }
// // // }
package frc.robot.commands.pathing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

public class NewPathing extends AutoFollower {
    private final Drive driveSubsystem;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.1;
    private int lastLookaheadIndex = 0;


    private final PID xPID = new PID(0.38, 0.0, 0.73);
    private final PID yPID = new PID(0.38, 0.0, 0.73);
    private final PID thetaPID = new PID(0.1, 0, 0.007);

    public NewPathing(List<PosePoint> points, Drive driveSubsystem) {
        this.points = points;
        this.driveSubsystem = driveSubsystem;
        xPID.setMinOutput(-3.0); xPID.setMaxOutput(3.0);
        yPID.setMinOutput(-3.0); yPID.setMaxOutput(3.0);
        thetaPID.setMinInput(-Math.PI); thetaPID.setMaxInput(Math.PI);
        thetaPID.setContinuous(true);
        thetaPID.setMinOutput(-0.5); thetaPID.setMaxOutput(0.5);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        lastLookaheadIndex = 0;
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
            0.0, 0.0, 0.0
        );

        Logger.recordOutput("Field/Robot", new double[] {
            robotPose.x, robotPose.y, robotPose.theta
        });

        int lookaheadIndex = lastLookaheadIndex;
        for (int i = lastLookaheadIndex; i < points.size(); i++) {
            double dx = robotPose.x - points.get(i).x;
            double dy = robotPose.y - points.get(i).y;
            double distance = Math.hypot(dx, dy);
            if (distance >= lookaheadDistance) {
                lookaheadIndex = i;
                break;
            }
        }

        if (lookaheadIndex >= points.size()) {
            lookaheadIndex = points.size() - 1;
        }
        if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) {
            lookaheadIndex++;
        }

        PosePoint target = points.get(lookaheadIndex);
        lastLookaheadIndex = lookaheadIndex;

        xPID.setSetPoint(target.x);
        yPID.setSetPoint(target.y);
        thetaPID.setSetPoint(-target.theta);

        double xCorrection = xPID.updatePID(robotPose.x);
        double yCorrection = yPID.updatePID(robotPose.y);
        double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

        double vxFF = target.dx * 0.1;
        double vyFF = target.dy * 0.1;

        double vx = vxFF + xCorrection;
        double vy = vyFF + yCorrection;
        double omega = thetaCorrection;

        // // double headingRad = robotPose.theta;
        // // double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        // // double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

        Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("theta error", thetaPID.getError());
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
        // Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
        Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

        driveSubsystem.driveSwerve(new Vector(vx, vy), omega);
    }
    // @Override
    // public void execute() {
    //     double currentTime = timer.get();
    //     PosePoint robotPose = new PosePoint(
    //         driveSubsystem.getX(),
    //         driveSubsystem.getY(),
    //         driveSubsystem.getAngle(),
    //         currentTime,
    //         0.0, 0.0, 0.0
    //     );
    
    //     Logger.recordOutput("Field/Robot", new double[] {
    //         robotPose.x, robotPose.y, robotPose.theta
    //     });
    
       
    //     int lookaheadIndex = lastLookaheadIndex;
    //     for (int i = lastLookaheadIndex; i < points.size(); i++) {
    //         double dx = robotPose.x - points.get(i).x;
    //         double dy = robotPose.y - points.get(i).y;
    //         double distance = Math.hypot(dx, dy);
    //         if (distance >= lookaheadDistance) {
    //             lookaheadIndex = i;
    //             break;
    //         }
    //     }
    
    //     if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;
    //     if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) lookaheadIndex++;
    
    //     PosePoint target = points.get(lookaheadIndex);
    //     lastLookaheadIndex = lookaheadIndex;
    
     
    //     xPID.setSetPoint(target.x);
    //     yPID.setSetPoint(target.y);
    //     thetaPID.setSetPoint(target.theta);
    
    //     double xCorrection = xPID.updatePID(robotPose.x);
    //     double yCorrection = yPID.updatePID(robotPose.y);
    //     double thetaCorrection = thetaPID.updatePID(robotPose.theta);
    
    
    //     double vxFF = target.dx * 0.1;
    //     double vyFF = target.dy * 0.1;
    
        
    //     double vx = vxFF + xCorrection;
    //     double vy = vyFF + yCorrection;
    //     double omega = thetaCorrection;
    
    //     double thetaError = thetaPID.getError();
    //     if (Math.abs(Math.toDegrees(thetaError)) < 3) omega = 0.0;
    //     double turnScale = Math.max(0.0, 1.0 - Math.abs(thetaError) / Math.PI);
    //     vx *= turnScale;
    //     vy *= turnScale;
    
        
    //     double headingRad = robotPose.theta;
    //     double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
    //     double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);
    
      
    //     Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
    //     Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
    //     Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
    //     Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
    //     Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);
    
    //     driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
    // }
//     @Override
// public void execute() {
//     double currentTime = timer.get();
//     PosePoint robotPose = new PosePoint(
//         driveSubsystem.getX(),
//         driveSubsystem.getY(),
//         driveSubsystem.getAngle(),
//         currentTime,
//         0.0, 0.0, 0.0
//     );

//     Logger.recordOutput("Field/Robot", new double[] {
//         robotPose.x, robotPose.y, robotPose.theta
//     });

//     int lookaheadIndex = lastLookaheadIndex;
//     for (int i = lastLookaheadIndex; i < points.size(); i++) {
//         double dx = robotPose.x - points.get(i).x;
//         double dy = robotPose.y - points.get(i).y;
//         double distance = Math.hypot(dx, dy);
//         if (distance >= lookaheadDistance) {
//             lookaheadIndex = i;
//             break;
//         }
//     }

//     if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;
//     if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) lookaheadIndex++;

//     PosePoint target = points.get(lookaheadIndex);
//     lastLookaheadIndex = lookaheadIndex;

//     xPID.setSetPoint(target.x);
//     yPID.setSetPoint(target.y);
//     thetaPID.setSetPoint(target.theta);

//     double xCorrection = xPID.updatePID(robotPose.x);
//     double yCorrection = yPID.updatePID(robotPose.y);
//     double thetaCorrection = thetaPID.updatePID(robotPose.theta);

//     double vxFF = target.dx * 0.1;
//     double vyFF = target.dy * 0.1;

//     double vx = vxFF + xCorrection;
//     double vy = vyFF + yCorrection;
//     double omega = thetaCorrection;

//     double thetaError = thetaPID.getError();
//     if (Math.abs(thetaError) < Math.toRadians(1)) omega = 0.0;

//     double headingRad = robotPose.theta;
//     double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//     double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//     Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
//     Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
//     Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
//     Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
//     Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

//     driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
// }

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
        return distanceToEnd < 0.05 && lastLookaheadIndex >= points.size() - 1;
    }

    @Override
    public int getPathPointIndex() {
        return lastLookaheadIndex;
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
        points.clear();
        for (int i = pointIndex; i <= toIndex; i++) {
            JSONObject point = pathJSON.getJSONObject(String.valueOf(i));
            double x = point.getDouble("x");
            double y = point.getDouble("y");
            double theta = point.getDouble("theta");
            double dx = point.optDouble("dx", 0.0);
            double dy = point.optDouble("dy", 0.0);
            double dtheta = point.optDouble("dtheta", 0.0);
            points.add(new PosePoint(x, y, theta, 0.0, dx, dy, dtheta));
        }
    }
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

// public class NewPathing extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.1;
//     private int lastLookaheadIndex = 0;

//     private final PID xPID = new PID(0.35, 0.0, 0.73);
//     private final PID yPID = new PID(0.35, 0.0, 0.73);
//     private final PID thetaPID = new PID(0.017, 0, 0.005);

//     public NewPathing(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;
//         xPID.setMinOutput(-3.0); xPID.setMaxOutput(3.0);
//         yPID.setMinOutput(-3.0); yPID.setMaxOutput(3.0);
//         thetaPID.setMinInput(-Math.PI); thetaPID.setMaxInput(Math.PI);
//         thetaPID.setContinuous(true);
//         thetaPID.setMinOutput(-0.5); thetaPID.setMaxOutput(0.5);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//         lastLookaheadIndex = 0;
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
//             0.0, 0.0, 0.0
//         );

//         Logger.recordOutput("Field/Robot", new double[] {
//             robotPose.x, robotPose.y, robotPose.theta
//         });

//         int lookaheadIndex = lastLookaheadIndex;
//         for (int i = lastLookaheadIndex; i < points.size(); i++) {
//             double dx = robotPose.x - points.get(i).x;
//             double dy = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dx, dy);
//             if (distance >= lookaheadDistance) {
//                 lookaheadIndex = i;
//                 break;
//             }
//         }

//         if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;
//         if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) lookaheadIndex++;

//         PosePoint target = points.get(lookaheadIndex);
//         lastLookaheadIndex = lookaheadIndex;

//         xPID.setSetPoint(target.x);
//         yPID.setSetPoint(target.y);
//         thetaPID.setSetPoint(target.theta);

//         double xCorrection = xPID.updatePID(robotPose.x);
//         double yCorrection = yPID.updatePID(robotPose.y);
//         double thetaCorrection = thetaPID.updatePID(robotPose.theta);

//         double vxFF = target.dx * 0.1;
//         double vyFF = target.dy * 0.1;

//         double vx = vxFF + xCorrection;
//         double vy = vyFF + yCorrection;
//         double omega = thetaCorrection;

//         double thetaError = thetaPID.getError();
//         if (Math.abs(Math.toDegrees(thetaError)) < 1.5) omega = 0.0;

//         double turnScale = Math.max(0.0, 1.0 - Math.abs(thetaError) / Math.PI);
//         vx *= turnScale;
//         vy *= turnScale;

//         double headingRad = robotPose.theta;
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
//         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
//         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
//         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
//         Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

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
//         return distanceToEnd < 0.05 && lastLookaheadIndex >= points.size() - 1;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return lastLookaheadIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
//         points.clear();
//         for (int i = pointIndex; i <= toIndex; i++) {
//             JSONObject point = pathJSON.getJSONObject(String.valueOf(i));
//             double x = point.getDouble("x");
//             double y = point.getDouble("y");
//             double theta = point.getDouble("theta");
//             double dx = point.optDouble("dx", 0.0);
//             double dy = point.optDouble("dy", 0.0);
//             double dtheta = point.optDouble("dtheta", 0.0);
//             points.add(new PosePoint(x, y, theta, 0.0, dx, dy, dtheta));
//         }
//     }
// }
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

// public class NewPathing extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.1;
//     private int lastLookaheadIndex = 0;

//     private final PID xPID = new PID(0.35, 0.0, 0.73);
//     private final PID yPID = new PID(0.35, 0.0, 0.73);
//     private final PID thetaPID = new PID(0.017, 0, 0.005);

//     public NewPathing(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;
//         xPID.setMinOutput(-3.0); xPID.setMaxOutput(3.0);
//         yPID.setMinOutput(-3.0); yPID.setMaxOutput(3.0);
//         thetaPID.setMinInput(-Math.PI); thetaPID.setMaxInput(Math.PI);
//         thetaPID.setContinuous(true);
//         thetaPID.setMinOutput(-0.5); thetaPID.setMaxOutput(0.5);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//         lastLookaheadIndex = 0;
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
//             0.0, 0.0, 0.0
//         );

//         Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});

//         int lookaheadIndex = lastLookaheadIndex;
//         for (int i = lastLookaheadIndex; i < points.size(); i++) {
//             double dx = robotPose.x - points.get(i).x;
//             double dy = robotPose.y - points.get(i).y;
//             double distance = Math.hypot(dx, dy);
//             if (distance >= lookaheadDistance) {
//                 lookaheadIndex = i;
//                 break;
//             }
//         }

//         if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;
//         if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) lookaheadIndex++;

//         PosePoint target = points.get(lookaheadIndex);
//         lastLookaheadIndex = lookaheadIndex;

//         xPID.setSetPoint(target.x);
//         yPID.setSetPoint(target.y);
//         thetaPID.setSetPoint(target.theta);

//         double xCorrection = xPID.updatePID(robotPose.x);
//         double yCorrection = yPID.updatePID(robotPose.y);
//         double thetaCorrection = thetaPID.updatePID(robotPose.theta);

//         double vxFF = target.dx * 0.1;
//         double vyFF = target.dy * 0.1;

//         double vx = vxFF + xCorrection;
//         double vy = vyFF + yCorrection;
//         double omega = thetaCorrection;

//         double thetaError = Math.abs(thetaPID.getError());

//         // small deadband for stopping small oscillations
//         if (Math.toDegrees(thetaError) < 0.5) omega = 0.0;

//         // soft translational scaling based on heading error (never fully stops)
//         double maxScale = 1.0;
//         double minScale = 0.5;
//         double headingScale = Math.max(minScale, maxScale - (Math.toDegrees(thetaError) / 90.0) * (maxScale - minScale));
//         vx *= headingScale;
//         vy *= headingScale;

//         double headingRad = robotPose.theta;
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
//         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
//         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
//         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
//         Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

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
//         return distanceToEnd < 0.05 && lastLookaheadIndex >= points.size() - 1;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return lastLookaheadIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
//         points.clear();
//         for (int i = pointIndex; i <= toIndex; i++) {
//             JSONObject point = pathJSON.getJSONObject(String.valueOf(i));
//             double x = point.getDouble("x");
//             double y = point.getDouble("y");
//             double theta = point.getDouble("theta");
//             double dx = point.optDouble("dx", 0.0);
//             double dy = point.optDouble("dy", 0.0);
//             double dtheta = point.optDouble("dtheta", 0.0);
//             points.add(new PosePoint(x, y, theta, 0.0, dx, dy, dtheta));
//         }
//     }
// }
