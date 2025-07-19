package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.tools.PathLoader;

import java.io.Console;
import java.util.List;

public class FollowPath extends Command {
    private final Drive drive;
    private final List<PathLoader.PosePoint> points;
    private final Timer timer = new Timer();
    private int currentIndex = 0;

    public FollowPath(List<PathLoader.PosePoint> inputPoints, Drive driveSubsystem) {
        this.points = inputPoints;
        this.drive = driveSubsystem;
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double now = timer.get() - 0.25;

        while (currentIndex < points.size() && now >= points.get(currentIndex).time) {

            PathLoader.PosePoint point = points.get(currentIndex);

            org.littletonrobotics.junction.Logger.recordOutput("Points X", point.x);
            org.littletonrobotics.junction.Logger.recordOutput("Points Y", point.y);
            org.littletonrobotics.junction.Logger.recordOutput("Points Theta", point.theta);

            Constants.x = point.x;
            Constants.y = point.y;
            Constants.angle = point.theta;

            currentIndex++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        boolean allPoints = currentIndex >= points.size();
        if (allPoints) {
            Constants.lastPoint = true;
        }
        return allPoints;
    }
}
