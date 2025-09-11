// // // // package frc.robot.commands.pathing;

// // // // import edu.wpi.first.wpilibj.Timer;
// // // // import edu.wpi.first.math.geometry.Pose2d;
// // // // import edu.wpi.first.math.geometry.Rotation2d;
// // // // import frc.robot.subsystems.Drive;
// // // // import frc.robot.tools.PathLoader.PosePoint;
// // // // import frc.robot.tools.math.Vector;
// // // // import frc.robot.tools.wrappers.AutoFollower;
// // // // import frc.robot.tools.math.PID;
// // // // import org.json.JSONObject;
// // // // import org.littletonrobotics.junction.Logger;
// // // // import java.util.List;

// // // // public class testPathingFF extends AutoFollower {
// // // //     private final Drive driveSubsystem;
// // // //     private final List<PosePoint> points;
// // // //     private final Timer timer = new Timer();
// // // //     private final double lookaheadDistance = 0.5;
// // // //     private int lastClosestIndex = 0;

// // // //     private final PID xPID = new PID(1.3, 0.0, 0.8);
// // // //     private final PID yPID = new PID(1.3, 0.0, 0.0);
// // // //     private final PID thetaPID = new PID(0.004, 0.0, 0.011);

// // // //     public testPathingFF(List<PosePoint> points, Drive driveSubsystem) {
// // // //         this.points = points;
// // // //         this.driveSubsystem = driveSubsystem;
// // // //         xPID.setMinOutput(-3.0);
// // // //         xPID.setMaxOutput(3.0);
// // // //         yPID.setMinOutput(-3.0);
// // // //         yPID.setMaxOutput(3.0);
// // // //         thetaPID.setMinInput(-180);
// // // //         thetaPID.setMaxInput(180);
// // // //         thetaPID.setContinuous(true);
// // // //         thetaPID.setMinOutput(-0.5);
// // // //         thetaPID.setMaxOutput(0.5);
// // // //     }

// // // //     @Override
// // // //     public void initialize() {
// // // //         timer.reset();
// // // //         timer.start();
// // // //         lastClosestIndex = 0;
// // // //         PosePoint start = points.get(0);
// // // //         driveSubsystem.resetOdometry(new Pose2d(start.x, start.y, new Rotation2d(start.theta)));
// // // //         double[][] pathPoses = new double[points.size()][3];
// // // //         for (int i = 0; i < points.size(); i++) {
// // // //             pathPoses[i][0] = points.get(i).x;
// // // //             pathPoses[i][1] = points.get(i).y;
// // // //             pathPoses[i][2] = points.get(i).theta;
// // // //         }
// // // //         Logger.recordOutput("Path", pathPoses);
// // // //     }

// // // //     @Override
// // // //     public void execute() {
// // // //         double currentTime = timer.get();
// // // //         PosePoint robotPose = new PosePoint(
// // // //             driveSubsystem.getX(),
// // // //             driveSubsystem.getY(),
// // // //             driveSubsystem.getAngle(),
// // // //             currentTime,
// // // //             0.0,
// // // //             0.0,
// // // //             0.0
// // // //         );

// // // //         Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});

// // // //         int closestIndex = lastClosestIndex;
// // // //         double closestDistance = Double.MAX_VALUE;
// // // //         for (int i = lastClosestIndex; i < points.size(); i++) {
// // // //             double dx = robotPose.x - points.get(i).x;
// // // //             double dy = robotPose.y - points.get(i).y;
// // // //             double distance = Math.hypot(dx, dy);
// // // //             if (distance < closestDistance) {
// // // //                 closestDistance = distance;
// // // //                 closestIndex = i;
// // // //             }
// // // //         }
// // // //         lastClosestIndex = closestIndex;

// // // //         int lookaheadIndex = closestIndex;
// // // //         for (int i = closestIndex; i < points.size(); i++) {
// // // //             double dx = robotPose.x - points.get(i).x;
// // // //             double dy = robotPose.y - points.get(i).y;
// // // //             double distance = Math.hypot(dx, dy);
// // // //             if (distance > lookaheadDistance) {
// // // //                 lookaheadIndex = i;
// // // //                 break;
// // // //             }
// // // //         }
// // // //         if (lookaheadIndex >= points.size()) {
// // // //             lookaheadIndex = points.size() - 1;
// // // //         }

// // // //         PosePoint target = points.get(lookaheadIndex);

// // // //         double xError = target.x - robotPose.x;
// // // //         double yError = target.y - robotPose.y;

// // // //         xPID.setSetPoint(0);
// // // //         yPID.setSetPoint(0);
// // // //         thetaPID.setSetPoint(Math.toDegrees(target.theta));

// // // //         double xCorrection = xPID.updatePID(-xError);
// // // //         double yCorrection = yPID.updatePID(-yError);
// // // //         double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

// // // //         double vxFF = target.dx;
// // // //         double vyFF = target.dy;
// // // //         double omegaFF = Math.toDegrees(target.dtheta);

// // // //         double kFF = 0.2;
// // // //         double vx = kFF * vxFF + xCorrection;
// // // //         double vy = kFF * vyFF + yCorrection;
// // // //         double omega = kFF * omegaFF + thetaCorrection;

// // // //         double headingRad = Math.toRadians(robotPose.theta);
// // // //         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
// // // //         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

// // // //         Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
// // // //         Logger.recordOutput("TargetPointIndex", lookaheadIndex);
// // // //         Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
// // // //         Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
// // // //         Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
// // // //         Logger.recordOutput("Velocities", new double [] { target.dx,target.dy,target.dtheta});

// // // //         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
// // // //     }

// // // //     @Override
// // // //     public void end(boolean interrupted) {
// // // //         driveSubsystem.stop();
// // // //     }

// // // //     @Override
// // // //     public boolean isFinished() {
// // // //         PosePoint lastPoint = points.get(points.size() - 1);
// // // //         double distx = driveSubsystem.getX() - lastPoint.x;
// // // //         double disty = driveSubsystem.getY() - lastPoint.y;
// // // //         double distanceToEnd = Math.hypot(distx, disty);
// // // //         return distanceToEnd < 0.05;
// // // //     }

// // // //     @Override
// // // //     public int getPathPointIndex() {
// // // //         return lastClosestIndex;
// // // //     }

// // // //     @Override
// // // //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// // // // }
// // // package frc.robot.commands.pathing;

// // // import edu.wpi.first.wpilibj.Timer;
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

// // // public class testPathingFF extends AutoFollower {
// // //     private final Drive driveSubsystem;
// // //     private final List<PosePoint> points;
// // //     private final Timer timer = new Timer();
// // //     private final double lookaheadDistance = 0.5;
// // //     private int lastClosestIndex = 0;

// // //     private final PID xPID = new PID(1.3, 0.0, 0.8);
// // //     private final PID yPID = new PID(1.3, 0.0, 0.0);
// // //     private final PID thetaPID = new PID(0.004, 0.0, 0.011);

// // //     public testPathingFF(List<PosePoint> points, Drive driveSubsystem) {
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

// // //         // 🔹 Fix: remove duplicate final point if it equals the first
// // //         if (points.size() > 1) {
// // //             PosePoint first = points.get(0);
// // //             PosePoint last = points.get(points.size() - 1);
// // //             if (Math.abs(first.x - last.x) < 1e-6 &&
// // //                 Math.abs(first.y - last.y) < 1e-6 &&
// // //                 Math.abs(first.theta - last.theta) < 1e-6) {
// // //                 points.remove(points.size() - 1);
// // //             }
// // //         }

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
// // //         if (lookaheadIndex >= points.size()) {
// // //             lookaheadIndex = points.size() - 1;
// // //         }

// // //         PosePoint target = points.get(lookaheadIndex);

// // //         double xError = target.x - robotPose.x;
// // //         double yError = target.y - robotPose.y;

// // //         xPID.setSetPoint(0);
// // //         yPID.setSetPoint(0);
// // //         thetaPID.setSetPoint(Math.toDegrees(target.theta));

// // //         double xCorrection = xPID.updatePID(-xError);
// // //         double yCorrection = yPID.updatePID(-yError);
// // //         double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

// // //         double vxFF = target.dx * 0.5;
// // //         double vyFF = target.dy * 0.5;
// // //         double omegaFF = Math.toDegrees(target.dtheta);

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
// // //         Logger.recordOutput("Velocities", new double [] { target.dx,target.dy,target.dtheta});

// // //         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
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
// // //         return distanceToEnd < 0.05;
// // //     }

// // //     @Override
// // //     public int getPathPointIndex() {
// // //         return lastClosestIndex;
// // //     }

// // //     @Override
// // //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// // // }
// // package frc.robot.commands.pathing;

// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Rotation2d;
// // import frc.robot.subsystems.Drive;
// // import frc.robot.tools.PathLoader.PosePoint;
// // import frc.robot.tools.math.Vector;
// // import frc.robot.tools.wrappers.AutoFollower;
// // import frc.robot.tools.math.PID;
// // import org.json.JSONObject;
// // import org.littletonrobotics.junction.Logger;
// // import java.util.List;

// // public class testPathingFF extends AutoFollower {
// //     private final Drive driveSubsystem;
// //     private final List<PosePoint> points;
// //     private final Timer timer = new Timer();
// //     private final double lookaheadDistance = 0.2;
// //     private int lastClosestIndex = 0;

// //     private final PID xPID = new PID(1.3, 0.0, 0.8);
// //     private final PID yPID = new PID(1.3, 0.0, 0.0);
// //     private final PID thetaPID = new PID(0.004, 0.0, 0.011);

// //     private final double finishThreshold = 0.05;

// //     public testPathingFF(List<PosePoint> points, Drive driveSubsystem) {
// //         this.points = points;
// //         this.driveSubsystem = driveSubsystem;
// //         xPID.setMinOutput(-3.0);
// //         xPID.setMaxOutput(3.0);
// //         yPID.setMinOutput(-3.0);
// //         yPID.setMaxOutput(3.0);
// //         thetaPID.setMinInput(-180);
// //         thetaPID.setMaxInput(180);
// //         thetaPID.setContinuous(true);
// //         thetaPID.setMinOutput(-0.5);
// //         thetaPID.setMaxOutput(0.5);
// //     }

// //     @Override
// //     public void initialize() {
// //         timer.reset();
// //         timer.start();
// //         lastClosestIndex = 0;
// //         PosePoint start = points.get(0);
// //         driveSubsystem.resetOdometry(new Pose2d(start.x, start.y, new Rotation2d(start.theta)));
// //         double[][] pathPoses = new double[points.size()][3];
// //         for (int i = 0; i < points.size(); i++) {
// //             pathPoses[i][0] = points.get(i).x;
// //             pathPoses[i][1] = points.get(i).y;
// //             pathPoses[i][2] = points.get(i).theta;
// //         }
// //         Logger.recordOutput("Path", pathPoses);
// //     }
// //     @Override
// // public void execute() {
// //     double currentTime = timer.get();
// //     PosePoint robotPose = new PosePoint(
// //         driveSubsystem.getX(),
// //         driveSubsystem.getY(),
// //         driveSubsystem.getAngle(),
// //         currentTime,
// //         0.0,
// //         0.0,
// //         0.0
// //     );

    
// //     int closestIndex = lastClosestIndex;
// //     double closestDistance = Double.MAX_VALUE;
// //     for (int i = lastClosestIndex; i < points.size(); i++) {
// //         double d = Math.hypot(robotPose.x - points.get(i).x, robotPose.y - points.get(i).y);
// //         if (d < closestDistance) {
// //             closestDistance = d;
// //             closestIndex = i;
// //         }
// //     }
// //     lastClosestIndex = closestIndex;

   
// //     PosePoint target = points.get(points.size() - 1);
// //     double accumulatedDistance = 0.0;
// //     for (int i = closestIndex; i < points.size() - 1; i++) {
// //         PosePoint p1 = points.get(i);
// //         PosePoint p2 = points.get(i + 1);
// //         double segment = Math.hypot(p2.x - p1.x, p2.y - p1.y);
// //         accumulatedDistance += segment;
// //         if (accumulatedDistance >= lookaheadDistance) {
// //             target = p2;
// //             break;
// //         }
// //     }

    
// //     double xError = target.x - robotPose.x;
// //     double yError = target.y - robotPose.y;

// //     xPID.setSetPoint(0);
// //     yPID.setSetPoint(0);
// //     thetaPID.setSetPoint(Math.toDegrees(target.theta));

// //     double xCorrection = xPID.updatePID(-xError);
// //     double yCorrection = yPID.updatePID(-yError);
// //     double thetaCorrection = -thetaPID.updatePID(robotPose.theta);

// //     double vxFF = target.dx * 0.5;
// //     double vyFF = target.dy * 0.5;
// //     double omegaFF = Math.toDegrees(target.dtheta);

// //     double vx = vxFF + xCorrection;
// //     double vy = vyFF + yCorrection;
// //     double omega = omegaFF + thetaCorrection;

// //     double headingRad = Math.toRadians(robotPose.theta);
// //     double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
// //     double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

// //     driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
// // }

// //     @Override
// //     public void end(boolean interrupted) {
// //         driveSubsystem.stop();
// //     }

// //     @Override
// //     public boolean isFinished() {
// //         if (lastClosestIndex >= points.size() - 1) {
// //             PosePoint lastPoint = points.get(points.size() - 1);
// //             double distx = driveSubsystem.getX() - lastPoint.x;
// //             double disty = driveSubsystem.getY() - lastPoint.y;
// //             double distanceToEnd = Math.hypot(distx, disty);
// //             return distanceToEnd < finishThreshold;
// //         }
// //         return false;
// //     }

// //     @Override
// //     public int getPathPointIndex() {
// //         return lastClosestIndex;
// //     }

// //     @Override
// //     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// // }
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

// public class testPathingFF extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private int lastIndex = 0;

//     private final PID xPID = new PID(1.3, 0.0, 0.8);
//     private final PID yPID = new PID(1.3, 0.0, 0.0);
//     private final PID thetaPID = new PID(0.004, 0.0, 0.011);

//     private final double finishThreshold = 0.05;

//     private final boolean fullSend = false;
//     private final boolean accurate = false;
//     private final double LOOKAHEAD_LINEAR_RADIUS = 1.0;
//     private final double LOOKAHEAD_ANGULAR_RADIUS = Math.toRadians(120.0);
//     private final double MIN_LOOKAHEAD = 0.05;
//     private final double BASE_LOOKAHEAD_GAIN = 0.35;
//     private final double FULL_SEND_LOOKAHEAD = 0.65;
//     private final double FEED_FORWARD_MULTIPLIER = 0.5;
//     private final double ACCURATE_END_FF = 0.25;

//     public testPathingFF(List<PosePoint> points, Drive driveSubsystem) {
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
//         lastIndex = 0;
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

//     private boolean insideRadius(double dx, double dy, double dtheta, double r) {
//         double e = Math.sqrt(dx * dx + dy * dy + dtheta * dtheta);
//         Logger.recordOutput("Error", e);
//         return e < r;
//     }

//     @Override
//     public void execute() {
//         double t = timer.get();
//         PosePoint robot = new PosePoint(
//             driveSubsystem.getX(),
//             driveSubsystem.getY(),
//             driveSubsystem.getAngle(),
//             t,
//             0.0,
//             0.0,
//             0.0
//         );

//         int targetIndex = points.size() - 1;
//         PosePoint target = points.get(points.size() - 1);

//         for (int i = Math.max(0, lastIndex); i < points.size(); i++) {
//             PosePoint p = points.get(i);
//             double linMag = Math.hypot(p.dx / LOOKAHEAD_LINEAR_RADIUS, p.dy / LOOKAHEAD_LINEAR_RADIUS);
//             double tgtMag = Math.hypot(linMag, p.dtheta / LOOKAHEAD_ANGULAR_RADIUS);
//             double lookahead = fullSend ? FULL_SEND_LOOKAHEAD : BASE_LOOKAHEAD_GAIN * tgtMag + MIN_LOOKAHEAD;

//             double dx = (robot.x - p.x) / LOOKAHEAD_LINEAR_RADIUS;
//             double dy = (robot.y - p.y) / LOOKAHEAD_LINEAR_RADIUS;
//             double dth = Math.toRadians(robot.theta) - p.theta;
//             while (dth > Math.PI) dth -= 2 * Math.PI;
//             while (dth < -Math.PI) dth += 2 * Math.PI;
//             dth = dth / LOOKAHEAD_ANGULAR_RADIUS;

//             if (!insideRadius(dx, dy, dth, lookahead)) {
//                 targetIndex = i;
//                 target = p;
//                 break;
//             }
//         }

//         lastIndex = targetIndex;

//         double targetThetaDeg = Math.toDegrees(target.theta);
//         while (targetThetaDeg - robot.theta > 180) targetThetaDeg -= 360;
//         while (targetThetaDeg - robot.theta < -180) targetThetaDeg += 360;

//         xPID.setSetPoint(target.x);
//         yPID.setSetPoint(target.y);
//         thetaPID.setSetPoint(targetThetaDeg);

//         xPID.updatePID(robot.x);
//         yPID.updatePID(robot.y);
//         thetaPID.updatePID(robot.theta);

//         double xVelNoFF = xPID.getResult();
//         double yVelNoFF = yPID.getResult();
//         double thetaVelNoFF = -thetaPID.getResult();

//         double f = accurate ? ACCURATE_END_FF : FEED_FORWARD_MULTIPLIER;
//         double feedForwardX = target.dx * f;
//         double feedForwardY = target.dy * f;
//         double feedForwardTheta = -Math.toDegrees(target.dtheta) * f;

//         double vx = xVelNoFF + feedForwardX;
//         double vy = yVelNoFF + feedForwardY;
//         double omega = thetaVelNoFF + feedForwardTheta;

//         double headingRad = Math.toRadians(robot.theta);
//         double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
//         double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

//         driveSubsystem.driveSwerve(new Vector(robotX, robotY), omega);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         if (lastIndex >= points.size() - 1) {
//             PosePoint lastPoint = points.get(points.size() - 1);
//             double dx = driveSubsystem.getX() - lastPoint.x;
//             double dy = driveSubsystem.getY() - lastPoint.y;
//             return Math.hypot(dx, dy) < finishThreshold;
//         }
//         return false;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return lastIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {}
// }
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

public class testPathingFF extends AutoFollower {
    private final Drive driveSubsystem;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.1;
    private int lastLookaheadIndex = 0;


    private final PID xPID = new PID(0.36, 0.0, 0.75);
    private final PID yPID = new PID(0.36, 0.0, 0.75);
    private final PID thetaPID = new PID(0.016, 0.0, 0.019);


    public testPathingFF(List<PosePoint> points, Drive driveSubsystem) {
        this.points = points;
        this.driveSubsystem = driveSubsystem;
        xPID.setMinOutput(-3.0); xPID.setMaxOutput(3.0);
        yPID.setMinOutput(-3.0); yPID.setMaxOutput(3.0);
        thetaPID.setMinInput(-180); thetaPID.setMaxInput(180); thetaPID.setContinuous(true);
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
    
        Logger.recordOutput("Field/Robot", new double[] {robotPose.x, robotPose.y, robotPose.theta});
    
        int lookaheadIndex = lastLookaheadIndex;
        for (int i = lastLookaheadIndex; i < points.size(); i++) {
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
        if (lookaheadIndex == lastLookaheadIndex && lookaheadIndex < points.size() - 1) {
            lookaheadIndex++;
        }
    
        PosePoint target = points.get(lookaheadIndex);
        lastLookaheadIndex = lookaheadIndex; 
    
        double xError = target.x - robotPose.x;
        double yError = target.y - robotPose.y;
    
        xPID.setSetPoint(0);
        yPID.setSetPoint(0);
        thetaPID.setSetPoint(Math.toDegrees(target.theta));
    
        double xCorrection = xPID.updatePID(-xError);
        double yCorrection = yPID.updatePID(-yError);
        double thetaCorrection = -thetaPID.updatePID(robotPose.theta);
    
        double vxFF = target.dx*0.1;
        double vyFF = target.dy*0.1;
        double omegaFF = Math.toDegrees(target.dtheta)*0.1;
    
        double vx = vxFF + xCorrection ;
        double vy = vyFF + yCorrection;
        double omega = omegaFF + thetaCorrection;
    
        double headingRad = Math.toRadians(omega);
        double robotX = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        double robotY = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);
    
        Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, omegaFF});
        Logger.recordOutput("CombinedOutput", new double[] {robotX, robotY, omega});
        Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);
    
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

