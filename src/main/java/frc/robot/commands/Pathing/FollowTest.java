// package frc.robot.commands.Pathing;

// import frc.robot.subsystems.Drive;
// import frc.robot.tools.math.Vector;
// import frc.robot.tools.math.PID;
// import frc.robot.tools.PathLoader.PosePoint;
// import frc.robot.tools.wrappers.AutoFollower;

// import java.util.List;

// import org.json.JSONObject;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.math.geometry.Pose2d;

// public class FollowTest extends AutoFollower {
//     private final Drive drive;
//     private final double lookaheadDistance = 0.3;

//     private final List<PosePoint> points;
//     private int currentIndex;
//     private final PID xPID;
//     private final PID yPID;
//     private final PID thetaPID;

//     private final Timer timer = new Timer();
//     private final double pathEndTime;

//     public FollowTest(List<PosePoint> points, Drive drive) {
//         this.points = points;
//         this.drive = drive;

//         xPID = new PID(4, 0.0, 2.1);
//         yPID = new PID(4, 0.0, 2.1);
//         thetaPID = new PID(2, 0.0, 2.9);
//         xPID.setMinOutput(-1.0);
//         xPID.setMaxOutput(1.0);
//         yPID.setMinOutput(-1.0);
//         yPID.setMaxOutput(1.0);
      

//         thetaPID.setContinuous(true);
//         thetaPID.setMinInput(-180);
//         thetaPID.setMaxInput(180);
        
//         currentIndex = 0;

//         if (points.isEmpty()) {
//             pathEndTime = 0;
//         } else {
//             pathEndTime = points.get(points.size() - 1).time;
//         }
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();
//         currentIndex = 0;
//     }

//     @Override
//     public void execute() {
//         if (points.isEmpty()) {
//             drive.stop();
//             return;
//         }

//         Pose2d currentPose = drive.getPose2D();
//         double timeNow = timer.get();

//         while (currentIndex < points.size() - 1 && points.get(currentIndex).time < timeNow) {
//             currentIndex++;
//         }

//         PosePoint lookahead = points.get(currentIndex);
//         for (int i = currentIndex; i < points.size(); i++) {
//             PosePoint p = points.get(i);
//             double dx = p.x - currentPose.getX();
//             double dy = p.y - currentPose.getY();
//             if (Math.hypot(dx, dy) >= lookaheadDistance) {
//                 lookahead = p;
//                 break;
//             }
//         }

//         xPID.setSetPoint(lookahead.x);
//         yPID.setSetPoint(lookahead.y);
//         thetaPID.setSetPoint(Math.toDegrees(lookahead.theta));

//         double xSpeed = xPID.updatePID(currentPose.getX());
//         double ySpeed = yPID.updatePID(currentPose.getY());
       

//         Vector fieldVector = new Vector(xSpeed, ySpeed);

//         drive.autoDrive(fieldVector, Math.toDegrees(lookahead.theta));
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drive.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return currentIndex >= points.size() - 1 && timer.get() > pathEndTime;
//     }

//     @Override
//     public int getPathPointIndex() {
//         return currentIndex;
//     }

//     @Override
//     public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'from'");
//     }
// }
