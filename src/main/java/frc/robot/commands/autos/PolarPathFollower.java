package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.tools.wrappers.AutoFollower;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.TriggerCommand;

public class PolarPathFollower extends ParallelCommandGroup {
    double time = 0;
    AutoFollower autoFollower;
    double startTime = 0;
    double endTime = 0;
    boolean timerStarted = false;
    JSONObject path;

    public PolarPathFollower(JSONObject path, HashMap<String, Supplier<Command>> commandMap,
            HashMap<String, BooleanSupplier> conditionMap) {
        startTime = path.getJSONArray("sampled_points").getJSONObject(0).getDouble("time");
        this.path = path;
    }

    public Command runCommand(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
            HashMap<String, BooleanSupplier> conditionMap) {
        if (command.has("command")) {
            return runSingleCommand(command, commandMap);
        } else if (command.has("branched_command")) {
            BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
            BooleanSupplier End = () -> command.getDouble("end") <= GetTime();
            JSONObject OnTrue = command.getJSONObject("branched_command").getJSONObject("on_true");
            JSONObject OnFalse = command.getJSONObject("branched_command").getJSONObject("on_false");
            BooleanSupplier condition = conditionMap
                    .get(command.getJSONObject("branched_command").getString("condition"));

            return new TriggerCommand(Start,
                    new ConditionalCommand(runCommand(OnTrue, commandMap, conditionMap),
                            runCommand(OnFalse, commandMap, conditionMap), condition),
                    End);

        } else {
            throw new IllegalArgumentException("Invalid command JSON: " + command.toString());
        }

    }

    public Command runSingleCommand(JSONObject command, HashMap<String, Supplier<Command>> commandMap) {
        Command Commands = commandMap.get(command.getJSONObject("command").getString("name")).get();
        BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
        BooleanSupplier End = () -> command.getDouble("end") <= GetTime();
        return new TriggerCommand(Start, Commands, End);
    }

    double GetTime() {
        if (autoFollower.isFinished() || !autoFollower.isScheduled()) {
            if (!timerStarted) {
                endTime = Timer.getFPGATimestamp();
                timerStarted = true;
            }
            time = Timer.getFPGATimestamp() - endTime;
        }

        if (autoFollower.isFinished()) {
            time += path.getJSONArray("sampled_points").getJSONObject(path.getJSONArray("time").length() - 1)
                    .getDouble("time");
        } else if (autoFollower.isScheduled()) {
            timerStarted = false;
            time = path.getJSONArray("sampled_points").getJSONObject(autoFollower.getPathPointIndex())
                    .getDouble("time");
        }

        return time;
    }

}
