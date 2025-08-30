// package frc.robot.commands.pathing;

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

// public class FixedPurePursuitAutoFollower extends AutoFollower {
//     private final Drive driveSubsystem;
//     private final List<PosePoint> points;
//     private final Timer timer = new Timer();
//     private final double lookaheadDistance = 0.3;
//     private int lastClosestIndex = 0;

//     private final PID xPID = new PID(0.0, 0.0, 0.0);
//     private final PID yPID = new PID(0.0, 0.0, 0.0);
//     private final PID thetaPID = new PID(1.0, 0.0, 0.0);

//     public FixedPurePursuitAutoFollower(List<PosePoint> points, Drive driveSubsystem) {
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
//         lastClosestIndex = 0;
//         PosePoint p = points.get(0);
//         System.out.println("Start: " + p.x + ", " + p.y + ", " + p.theta);

//         double[][] pathPoses = new double[points.size()][3];
//         for (int i = 0; i < points.size(); i++) {
//             pathPoses[i][0] = points.get(i).x;
//             pathPoses[i][1] = points.get(i).y;
//             pathPoses[i][2] = points.get(i).theta;
//         }

//         Logger.recordOutput("Path", pathPoses);
//     }

//     private double normalizeDegrees(double angle) {
//         while (angle > 180) angle -= 360;
//         while (angle < -180) angle += 360;
//         return angle;
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

//         Logger.recordOutput("Field/Robot", new double[]{
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

//         double normalizedTargetTheta = normalizeDegrees(Math.toDegrees(target.theta));
//         double normalizedRobotTheta = normalizeDegrees(robotPose.theta);

//         xPID.setSetPoint(target.x);
//         yPID.setSetPoint(target.y);
//         thetaPID.setSetPoint(normalizedTargetTheta);

//         double xOutput = xPID.updatePID(robotPose.x);
//         double yOutput = yPID.updatePID(robotPose.y);
//         double turnOutput = thetaPID.updatePID(normalizedRobotTheta) / 8.0;

//         double headingRad = Math.toRadians(robotPose.theta);
//         double robotX = xOutput * Math.cos(-headingRad) - yOutput * Math.sin(-headingRad);
//         double robotY = xOutput * Math.sin(-headingRad) + yOutput * Math.cos(-headingRad);

//         Logger.recordOutput("Target", new double[]{target.x, target.y, target.theta});
//         Logger.recordOutput("PIDOutput", new double[]{xOutput, yOutput, turnOutput});

//         driveSubsystem.autoDrive(new Vector(robotX, robotY), turnOutput);
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
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
//     }
// }
