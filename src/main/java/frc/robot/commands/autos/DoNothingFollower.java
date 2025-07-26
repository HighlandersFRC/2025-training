package frc.robot.commands.autos;

import org.json.JSONArray;
import org.json.JSONObject;

import frc.robot.tools.wrappers.AutoFollower;
import edu.wpi.first.wpilibj.Timer;
// Remove this line if it's not used anywhere in the file
import org.json.JSONObject;
import frc.robot.commands.autos.PolarPathFollower;

public class DoNothingFollower extends AutoFollower {
    double start = 0;
    double currentTime = 0;
    int index = 0;
    JSONArray path;
    boolean isFinished = false;

    public DoNothingFollower(JSONArray path) {
        this.path = path;
    }

    @Override
    public int getPathPointIndex() {
        // Since this is a "Do Nothing" follower, return a default value
        if (index <= path.length()) {

            long index = Math.round(currentTime / 0.01);
            this.index = (int) index;
            return (int) index;
        } else {
            throw new UnsupportedOperationException("Index out of bounds for path points.");
        }
    }

    @Override
    public void initialize() {
        isFinished = false;
        index = 0; // Reset index at the start
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (index >= path.length()) {
            isFinished = true; // Mark as finished if index exceeds path length
            return; // Prevents out-of-bounds access
        }
        currentTime = Timer.getFPGATimestamp() - start;

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
    }

    @Override
    public boolean isFinished() {
        return isFinished; // Return true if the path has been fully traversed
    }

}
