// package frc.robot.commands.Pathing;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drive;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.math.Vector;
// import frc.robot.tools.math.PID;
// import org.littletonrobotics.junction.Logger;

// import java.util.List;

// public class PurePursuit extends Command {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.25;
//     private int lastClosestIndex = 0;

//     private final PID xPID = new PID(4, 0.0, 2.1);
//     private final PID yPID = new PID(4, 0.0, 2.1);
//     private final PID thetaPID = new PID(2, 0.0, 2.9);

//     public PurePursuit(List<PosePoint> points, Drive driveSubsystem) {
//         this.points = points;
//         this.driveSubsystem = driveSubsystem;

//         xPID.setMinOutput(-3.0);
//         xPID.setMaxOutput(3.0);

//         yPID.setMinOutput(-3.0);
//         yPID.setMaxOutput(3.0);

//         thetaPID.setMinInput(-180);
//         thetaPID.setMaxInput(180);

//         thetaPID.setContinuous(true);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();

//         PosePoint p = points.get(0);
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
//        double currentTime = timer.get();

//         PosePoint robotPose = new PosePoint(
//             driveSubsystem.getX(),
//             driveSubsystem.getY(),
//             driveSubsystem.getAngle(),
//             currentTime
//         );

//         Logger.recordOutput("Field/Robot", new double[]{
//             robotPose.x , robotPose.y , robotPose.theta
//         });

//         int closestIndex = lastClosestIndex;
//         double closestDistance = Double.MAX_VALUE;

//         for (int i = lastClosestIndex; i < points.size(); i++) {
//             double dx = robotPose.x - points.get(i).x;
//             double dy = robotPose.y - points.get(i).y;
//             double distance = Math.sqrt(dx * dx + dy * dy);
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

//         PosePoint target = points.get(lookaheadIndex);

//         Logger.recordOutput("Target", new double[]{
//             target.x, target.y, target.theta
//         });
//         Logger.recordOutput("ClosestIndex", closestIndex);
//         Logger.recordOutput("LookaheadIndex", lookaheadIndex);
//         Logger.recordOutput("ElapsedTime", currentTime);



//         xPID.setSetPoint(target.x);
//         yPID.setSetPoint(target.y);
//         thetaPID.setSetPoint(target.theta);

//         double xOutput = xPID.updatePID(robotPose.x);
//         double yOutput = yPID.updatePID(robotPose.y);
//         double turnOutput = thetaPID.updatePID(robotPose.theta);

//         driveSubsystem.autoDrive(new Vector(xOutput, yOutput), turnOutput);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return lastClosestIndex >= points.size() - 1;
//     }
// }

// // package frc.robot.commands.Pathing;

// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.Drive;
// // import frc.robot.tools.PathLoader.PosePoint;
// // import frc.robot.tools.math.Vector;
// // import frc.robot.tools.math.PID;
// // import org.littletonrobotics.junction.Logger;

// // import java.util.List;

// // public class PurePursuit extends Command {
// //     private final Drive driveSubsystem;
// //     private final List<PosePoint> points;
// //     private final Timer timer = new Timer();
// //     private final double lookaheadDistance = 0.35;
// //     private int lastClosestIndex = 0;

// //     private final PID xPID = new PID(1.0, 0.0, 0.1);
// //     private final PID yPID = new PID(1.0, 0.0, 0.1);
// //     private final PID thetaPID = new PID(1.5, 0.0, 0.05);

// //     public PurePursuit(List<PosePoint> points, Drive driveSubsystem) {
// //         this.points = points;
// //         this.driveSubsystem = driveSubsystem;

// //         xPID.setMinOutput(-2.0);
// //         xPID.setMaxOutput(2.0);

// //         yPID.setMinOutput(-2.0);
// //         yPID.setMaxOutput(2.0);

// //         thetaPID.setMinInput(-180);
// //         thetaPID.setMaxInput(180);
// //         thetaPID.setContinuous(true);
// //     }

// //     @Override
// //     public void initialize() {
// //         timer.reset();
// //         timer.start();
// //         lastClosestIndex = 0;
// //         Logger.recordOutput("Path", points.stream().map(p -> new double[]{p.x, p.y, p.theta}).toArray(double[][]::new));
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

// //         Logger.recordOutput("Field/Robot", new double[]{robotPose.x, robotPose.y, robotPose.theta});

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
// //         PosePoint target = points.get(Math.min(lookaheadIndex, points.size() - 1));

// //         Logger.recordOutput("Target", new double[]{target.x, target.y, target.theta});
// //         Logger.recordOutput("LookaheadIndex", lookaheadIndex);
// //         Logger.recordOutput("ElapsedTime", currentTime);

// //         xPID.setSetPoint(target.x);
// //         yPID.setSetPoint(target.y);
// //         thetaPID.setSetPoint(target.theta);

// //         double xOutput = xPID.updatePID(robotPose.x);
// //         double yOutput = yPID.updatePID(robotPose.y);
// //         double turnOutput = thetaPID.updatePID(robotPose.theta);

// //         double headingRad = Math.toRadians(robotPose.theta);
// //         double robotX = xOutput * Math.cos(-headingRad) - yOutput * Math.sin(-headingRad);
// //         double robotY = xOutput * Math.sin(-headingRad) + yOutput * Math.cos(-headingRad);

// //         driveSubsystem.autoDrive(new Vector(robotX, robotY), turnOutput);
// //     }

// //     @Override
// //     public void end(boolean interrupted) {
// //         driveSubsystem.stop();
// //     }

// //     @Override
// //     public boolean isFinished() {
// //         return lastClosestIndex >= points.size() - 1;
// //     }
// // }
