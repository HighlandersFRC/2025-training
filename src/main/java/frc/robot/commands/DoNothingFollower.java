package frc.robot.commands;

import org.json.JSONArray;
import org.json.JSONObject;

import frc.robot.tools.wrappers.AutoFollower;
import edu.wpi.first.wpilibj.Timer;

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
        // Clamp index to valid range
        int maxIndex = path.length() - 1;
        long computedIndex = Math.round(currentTime / 0.01);
        int safeIndex = (int) Math.max(0, Math.min(computedIndex, maxIndex));
        this.index = safeIndex;
        return safeIndex;
    }

    @Override
    public void initialize() {
        isFinished = false;
        index = 0;
        start = Timer.getFPGATimestamp();
        currentTime = 0;
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp() - start;
        int idx = getPathPointIndex();
        if (idx >= path.length() - 1) {
            isFinished = true;
        }
        System.out.println(idx);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
        // No action for DoNothingFollower
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}