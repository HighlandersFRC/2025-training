package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                autoFollower = new DoNothingFollower(path.getJSONArray("sampledPoints"));
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
                        BooleanSupplier condition = conditionMap.get(
                                        command.getJSONObject("branched_command").getString("condition"));

                        return new TriggerCommand(
                                        Start,
                                        new ConditionalCommand(
                                                        runCommand(OnTrue, commandMap, conditionMap),
                                                        runCommand(OnFalse, commandMap, conditionMap),
                                                        condition),
                                        End);

                } else if (command.has("parallel_command_group")) {
                        ArrayList<Command> commands = new ArrayList<>();
                        for (int i = 0; i < command.getJSONObject("parallel_command_group")
                                        .getJSONArray("commands").length(); i++) {
                                commands.add(runCommand(
                                                command.getJSONObject("parallel_command_group")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap, conditionMap));
                        }
                        BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
                        BooleanSupplier End = () -> command.getDouble("end") <= GetTime();

                        return new TriggerCommand(
                                        Start,
                                        new ParallelCommandGroup(commands.toArray(new Command[0])),
                                        End);

                } else if (command.has("sequential_command_group")) {
                        ArrayList<Command> commands = new ArrayList<>();
                        for (int i = 0; i < command.getJSONObject("sequential_command_group")
                                        .getJSONArray("commands").length(); i++) {
                                commands.add(runCommand(
                                                command.getJSONObject("sequential_command_group")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap, conditionMap));
                        }
                        BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
                        BooleanSupplier End = () -> command.getDouble("end") <= GetTime();

                        return new TriggerCommand(
                                        Start,
                                        new SequentialCommandGroup(commands.toArray(new Command[0])),
                                        End);

                } else if (command.has("parallel_deadline_group")) {
                        Command deadlineCommand = runCommand(
                                        command.getJSONObject("parallel_deadline_group")
                                                        .getJSONArray("deadline_command").getJSONObject(0),
                                        commandMap, conditionMap);

                        ArrayList<Command> commands = new ArrayList<>();
                        for (int i = 0; i < command.getJSONObject("parallel_deadline_group")
                                        .getJSONArray("commands").length(); i++) {
                                commands.add(runCommand(
                                                command.getJSONObject("commands")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap, conditionMap));
                        }

                        Command[] otherCommands = commands.subList(1, commands.size()).toArray(new Command[0]);
                        BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
                        BooleanSupplier End = () -> command.getDouble("end") <= GetTime();

                        return new TriggerCommand(
                                        Start,
                                        new ParallelDeadlineGroup(deadlineCommand, commands.toArray(new Command[0])),
                                        End);

                } else if (command.has("parallel_race_group")) {
                        ArrayList<Command> commands = new ArrayList<>();

                        // Parse all commands in the race group
                        for (int i = 0; i < command.getJSONObject("parallel_race_group")
                                        .getJSONArray("commands").length(); i++) {
                                commands.add(runCommand(
                                                command.getJSONObject("parallel_race_group")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap, conditionMap));
                        }

                        BooleanSupplier Start = () -> command.getDouble("start") > GetTime();
                        BooleanSupplier End = () -> command.getDouble("end") <= GetTime();

                        return new TriggerCommand(
                                        Start,
                                        new ParallelRaceGroup(commands.toArray(new Command[0])),
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
                        time += path.getJSONArray("sampled_points")
                                        .getJSONObject(path.getJSONArray("time").length() - 1)
                                        .getDouble("time");
                } else if (autoFollower.isScheduled()) {
                        timerStarted = false;
                        time = path.getJSONArray("sampled_points").getJSONObject(autoFollower.getPathPointIndex())
                                        .getDouble("time");
                }

                return time;
        }

}
