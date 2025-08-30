

package frc.robot.commands.pathing;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drive;
import frc.robot.tools.PathLoader.PosePoint;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.tools.math.PID;
import org.littletonrobotics.junction.Logger;
import java.util.List;

public class FFTest extends AutoFollower {
    private final Drive drive;
    private final List<PosePoint> points;
    private final Timer timer = new Timer();
    private final double lookaheadDistance = 0.3;
    private int lastClosestIndex = 0;

    private final PID xPID = new PID(1.65, 0.0, 4.3);
    private final PID yPID = new PID(1.65, 0.0, 4.3);
    private final PID thetaPID = new PID(0.0048, 0.0, 0.011);

    public FFTest(List<PosePoint> points, Drive drive) {
        this.points = points;
        this.drive = drive;

        xPID.setMinOutput(-3.0);
        xPID.setMaxOutput(3.0);
        yPID.setMinOutput(-3.0);
        yPID.setMaxOutput(3.0);

        thetaPID.setContinuous(true);
        thetaPID.setMinInput(-180);
        thetaPID.setMaxInput(180);
        thetaPID.setMinOutput(-0.5);
        thetaPID.setMaxOutput(0.5);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        lastClosestIndex = 0;
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        double robotX = drive.getX();
        double robotY = drive.getY();
        double robotThetaDeg = Math.toDegrees(drive.getAngle());

        int closestIndex = lastClosestIndex;
        double closestDist = Double.MAX_VALUE;
        for (int i = lastClosestIndex; i < points.size(); i++) {
            double dx = robotX - points.get(i).x;
            double dy = robotY - points.get(i).y;
            double dist = Math.hypot(dx, dy);
            if (dist < closestDist) {
                closestDist = dist;
                closestIndex = i;
            }
        }
        lastClosestIndex = closestIndex;

        int lookaheadIndex = closestIndex;
        for (int i = closestIndex; i < points.size(); i++) {
            double dx = robotX - points.get(i).x;
            double dy = robotY - points.get(i).y;
            if (Math.hypot(dx, dy) >= lookaheadDistance) {
                lookaheadIndex = i;
                break;
            }
        }
        if (lookaheadIndex >= points.size()) lookaheadIndex = points.size() - 1;

        PosePoint target = points.get(lookaheadIndex);
        PosePoint prev = points.get(Math.max(lookaheadIndex - 1, 0));

        double dt = target.time - prev.time;
        if (dt < 0.02) dt = 0.02;

        
        double vxPath = (target.x - prev.x) / dt;
        double vyPath = (target.y - prev.y) / dt;

        
        
        double vxFF = vxPath * 0.5;
        double vyFF = vyPath * 0.5;

        double xError = target.x - robotX;
        double yError = target.y - robotY;

        xPID.setSetPoint(0);
        yPID.setSetPoint(0);
        thetaPID.setSetPoint(Math.toDegrees(target.theta));

        double xCorrection = xPID.updatePID(-xError);
        double yCorrection = yPID.updatePID(-yError);
        double thetaCorrection = thetaPID.updatePID(robotThetaDeg);

        double vx = vxFF + xCorrection;
        double vy = vyFF + yCorrection;

        double headingRad = Math.toRadians(drive.getAngle());
        double robotXVel = vx * Math.cos(-headingRad) - vy * Math.sin(-headingRad);
        double robotYVel = vx * Math.sin(-headingRad) + vy * Math.cos(-headingRad);

        drive.driveSwerve(new Vector(robotXVel, robotYVel), thetaCorrection);

        Logger.recordOutput("Target", new double[] {target.x, target.y, Math.toDegrees(target.theta)});
        Logger.recordOutput("PIDOutput", new double[] {xCorrection, yCorrection, thetaCorrection});
        Logger.recordOutput("VelocityFF", new double[] {vxFF, vyFF, 0});
        Logger.recordOutput("CombinedOutput", new double[] {robotXVel, robotYVel, thetaCorrection});
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
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
    public void from(int pointIndex, org.json.JSONObject pathJSON, int toIndex) {}
}
