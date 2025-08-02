package frc.robot.commands;

import org.json.JSONArray;
import org.json.JSONObject;

import frc.robot.tools.wrappers.AutoFollower;
import edu.wpi.first.wpilibj.Timer;

import org.json.JSONObject;

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
        index = 0;
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (index >= path.length()) {
            isFinished = true;
            return;
        }
        currentTime = Timer.getFPGATimestamp() - start;
        System.out.println(getPathPointIndex());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}