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

public class FinalPathing extends AutoFollower {
    private final Drive driveSubsystem;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.1;
    private int lastLookaheadIndex = 0;


    private final PID xPID = new PID(0.65, 0.0, 0.87);
    private final PID yPID = new PID(0.33, 0.0, 0.5);
    private final PID thetaPID = new PID(0.16, 0, 0.00);

    public FinalPathing(List<PosePoint> points, Drive driveSubsystem) {
        this.points = points;
        this.driveSubsystem = driveSubsystem;
        xPID.setMinOutput(-3.0); 
        xPID.setMaxOutput(3.0);
        yPID.setMinOutput(-3.0);
        yPID.setMaxOutput(3.0);
        thetaPID.setMinInput(-Math.PI);
        thetaPID.setMaxInput(Math.PI);
        thetaPID.setContinuous(true);
        thetaPID.setMinOutput(-0.5);
        thetaPID.setMaxOutput(0.5);
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

        Logger.recordOutput("Target", new double[] {target.x, target.y, target.theta});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("theta error", thetaPID.getError());
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF});
        Logger.recordOutput("CombinedOutput", new double[] {vx, vy, omega});
        Logger.recordOutput("LookaheadPointIndex", lookaheadIndex);

        driveSubsystem.driveSwerve(new Vector(vx, vy), omega);
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
        double headingError = Math.abs(thetaPID.getError());
        return distanceToEnd < 0.05
            && headingError < Math.toRadians(5.0)
            && lastLookaheadIndex >= points.size() - 1;
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