package frc.robot.commands;

import org.json.JSONObject;

import frc.robot.tools.wrappers.AutoFollower;

public class Print extends AutoFollower {

    @Override
    public int getPathPointIndex() {
        return 0; // This method should return the index of the current path point.
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println("Executing Print Command");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void from(int pointIndex, JSONObject pathJSON, int toIndex) {

    }

    @Override
    public boolean isFinished() {
        return false; // This method should return true when the command is finished.
    }

}
