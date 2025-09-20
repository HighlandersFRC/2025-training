// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.wrappers.AutoFollower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PolarPathFollower extends ParallelCommandGroup {
        /** Creates a new PolarPathFollower. */
        AutoFollower follower;
        VariableSpeedFollower defaultFollower;
        double endTime = 0, startTime = 0;
        boolean timerStarted = false;
        JSONObject pathJSON;
        TriggerCommand followerCommand;

        public PolarPathFollower(Drive drive, Peripherals peripherals, JSONObject pathJSON,
                        HashMap<String, Supplier<Command>> commandMap, HashMap<String, BooleanSupplier> conditionMap) {
                defaultFollower = new VariableSpeedFollower(drive, pathJSON.getJSONArray("sampled_points"),
                                false);
                startTime = pathJSON.getJSONArray("sampled_points").getJSONObject(0).getDouble("time");
                follower = defaultFollower;
                followerCommand = new TriggerCommand(
                                () -> startTime <= getPathTime(),
                                follower,
                                () -> false);
                this.pathJSON = pathJSON;
                ArrayList<Command> commands = new ArrayList<>();
                commands.add(followerCommand);
                for (int i = 0; i < pathJSON.getJSONArray("commands").length(); i++) {
                        JSONObject command = pathJSON.getJSONArray("commands").getJSONObject(i);
                        commands.add(addCommandsFromJSON(command, commandMap, conditionMap));
                }
                for (Command command : commands) {
                        addCommands(command);
                }
        }

        /**
         * This function processes a JSON object representing a command and returns a
         * corresponding {@link Command} object.
         * The function supports various command types such as single commands, branched
         * commands, parallel command groups,
         * parallel deadline groups, parallel race groups, and sequential command
         * groups.
         *
         * @param command      The JSON object representing the command.
         * @param commandMap   A map of command names to their corresponding
         *                     {@link Supplier} objects.
         * @param conditionMap A map of condition names to their corresponding
         *                     {@link BooleanSupplier} objects.
         * @return A {@link Command} object representing the processed command.
         * @throws IllegalArgumentException If the command JSON is invalid.
         */
        private Command addCommandsFromJSON(JSONObject command, HashMap<String, Supplier<Command>> commandMap,
                        HashMap<String, BooleanSupplier> conditionMap) {
                if (command.has("command")) {
                        return singleCommandFromJSON(command, commandMap);
                } else if (command.has("branched_command")) {
                        BooleanSupplier startSupplier = () -> command.getDouble("start") < getPathTime();
                        BooleanSupplier endSupplier = () -> command.getDouble("end") <= getPathTime();
                        JSONObject onTrue = command.getJSONObject("branched_command").getJSONObject("on_true");
                        JSONObject onFalse = command.getJSONObject("branched_command").getJSONObject("on_true");
                        BooleanSupplier condition = conditionMap
                                        .get(command.getJSONObject("branched_command").getString("condition"));
                        return new TriggerCommand(
                                        startSupplier,
                                        new ConditionalCommand(
                                                        addCommandsFromJSON(onTrue, commandMap, conditionMap),
                                                        addCommandsFromJSON(onFalse, commandMap, conditionMap),
                                                        condition),
                                        endSupplier);
                } else if (command.has("parallel_command_group")) {
                        ArrayList<Command> commands = new ArrayList<Command>();
                        for (int i = 0; i < command.getJSONObject("parallel_command_group").getJSONArray("commands")
                                        .length(); i++) {
                                commands.add(addCommandsFromJSON(
                                                command.getJSONObject("parallel_command_group").getJSONArray("commands")
                                                                .getJSONObject(i),
                                                commandMap,
                                                conditionMap));
                        }
                        return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
                                        new ParallelCommandGroup(
                                                        commands.toArray(new Command[0])),
                                        () -> command.getDouble("end") <= getPathTime());
                } else if (command.has("parallel_deadline_group")) {
                        Command deadlineCommand = addCommandsFromJSON(
                                        command.getJSONObject("parallel_deadline_group").getJSONArray("commands")
                                                        .getJSONObject(0),
                                        commandMap,
                                        conditionMap);
                        ArrayList<Command> commands = new ArrayList<Command>();
                        for (int i = 1; i < command.getJSONObject("parallel_deadline_group").getJSONArray("commands")
                                        .length(); i++) {
                                commands.add(addCommandsFromJSON(
                                                command.getJSONObject("parallel_deadline_group")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap,
                                                conditionMap));
                        }
                        return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
                                        new ParallelDeadlineGroup(
                                                        deadlineCommand,
                                                        commands.toArray(new Command[0])),
                                        () -> command.getDouble("end") <= getPathTime());
                } else if (command.has("parallel_command_group")) {
                        ArrayList<Command> commands = new ArrayList<Command>();
                        for (int i = 0; i < command.getJSONObject("parallel_race_group").getJSONArray("commands")
                                        .length(); i++) {
                                commands.add(addCommandsFromJSON(
                                                command.getJSONObject("parallel_race_group").getJSONArray("commands")
                                                                .getJSONObject(i),
                                                commandMap,
                                                conditionMap));
                        }
                        return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
                                        new ParallelRaceGroup(
                                                        commands.toArray(new Command[0])),
                                        () -> command.getDouble("end") <= getPathTime());
                } else if (command.has("sequential_command_group")) {
                        ArrayList<Command> commands = new ArrayList<Command>();
                        for (int i = 0; i < command.getJSONObject("sequential_command_group").getJSONArray("commands")
                                        .length(); i++) {
                                commands.add(addCommandsFromJSON(
                                                command.getJSONObject("sequential_command_group")
                                                                .getJSONArray("commands").getJSONObject(i),
                                                commandMap,
                                                conditionMap));
                        }
                        return new TriggerCommand(() -> command.getDouble("start") < getPathTime(),
                                        new SequentialCommandGroup(
                                                        commands.toArray(new Command[0])),
                                        () -> command.getDouble("end") <= getPathTime());
                } else {
                        throw new IllegalArgumentException("Invalid command JSON: " + command.toString());
                }
        }

        /**
         * This function processes a single command from a JSON object and returns a
         * corresponding {@link Command} object.
         * It handles the creation of various command types such as single commands,
         * with custom start and end times.
         * If the command is a {@link AutoFollower} instance, it sets up runnables
         * to
         * cancel and re-initialize the follower
         * at the specified start and end times.
         *
         * @param command    The JSON object representing the command. It should contain
         *                   the command name and start/end times.
         * @param commandMap A map of command names to their corresponding
         *                   {@link Supplier} objects.
         * @return A {@link Command} object representing the processed command.
         */
        private Command singleCommandFromJSON(JSONObject command, HashMap<String, Supplier<Command>> commandMap) {
                Command runner = commandMap.get(command.getJSONObject("command").getString("name")).get();
                BooleanSupplier startSupplier = () -> command.getDouble("start") < getPathTime();
                BooleanSupplier endSupplier = () -> command.getDouble("end") <= getPathTime();

                if (runner instanceof AutoFollower) {
                        Runnable cancelPathFollower = new Runnable() {
                                public void run() {
                                        int runFrom = getPointIndexFromTime(command.getDouble("start"));
                                        int runTo = getPointIndexFromTime(command.getDouble("end"));
                                        System.out.println("run from " + runFrom);
                                        System.out.println("run to " + runTo);
                                        follower.cancel();
                                        follower = (AutoFollower) runner;
                                        follower.from(runFrom, pathJSON, runTo);
                                }
                        };
                        Runnable cancelRunner = new Runnable() {
                                public void run() {
                                        int runFrom = getPointIndexFromTime(command.getDouble("end"));
                                        int runTo = pathJSON.getJSONArray("sampled_points").length() - 1;
                                        follower = defaultFollower;
                                        follower.from(runFrom, pathJSON, runTo);
                                }
                        };
                        Runnable nullRunnable = new Runnable() {
                                public void run() {
                                }
                        };
                        Consumer<Boolean> nullConsumer = new Consumer<Boolean>() {
                                @Override
                                public void accept(Boolean t) {
                                }
                        };
                        return new TriggerCommand(startSupplier, new SequentialCommandGroup(
                                        new FunctionalCommand(cancelPathFollower, nullRunnable, nullConsumer,
                                                        () -> true),
                                        new TriggerCommand(() -> true, runner, endSupplier),
                                        new FunctionalCommand(cancelRunner, nullRunnable, nullConsumer, () -> true),
                                        new TriggerCommand(() -> true, defaultFollower,
                                                        () -> false)),
                                        () -> false);
                }

                return new TriggerCommand(startSupplier, runner, endSupplier);
        }

        /**
         * This function calculates and returns the current time along the path based on
         * the {@link AutoFollower} follower's state.
         * If the follower is finished or not scheduled, it calculates the time based on
         * the elapsed time since the end of the path.
         * If the follower is still running, it calculates the time based on the time of
         * the current path point.
         *
         * @return The current time along the path in seconds.
         */
        double getPathTime() {
                double retval;
                if (follower.isFinished() || !follower.isScheduled()) {
                        if (!timerStarted) {
                                endTime = Timer.getFPGATimestamp();
                                timerStarted = true;
                        }
                        retval = Timer.getFPGATimestamp() - endTime;
                        if (follower.isFinished()) {
                                retval += pathJSON.getJSONArray("sampled_points")
                                                .getJSONObject(pathJSON.getJSONArray("sampled_points").length() - 1)
                                                .getDouble("time");
                        }
                } else {
                        timerStarted = false;
                        retval = pathJSON.getJSONArray("sampled_points")
                                        .getJSONObject(follower.getPathPointIndex()).getDouble("time");
                }
                // Logger.recordOutput("Path Time", retval);
                // Logger.recordOutput("Path Start Time", defaultFollower.pathStartTime);
                return retval;
        }

        /**
         * This function calculates and returns the index of the path point that
         * corresponds to the given time.
         * It iterates through the path points, comparing the given time with the time
         * of each point.
         * If the given time is greater than or equal to the time of a point, the
         * function returns the index of that point.
         * If the given time is greater than the time of all points, the function
         * returns the index of the last point.
         *
         * @param time The time along the path for which the corresponding path point
         *             index needs to be found.
         * @return The index of the path point that corresponds to the given time.
         */
        private int getPointIndexFromTime(double time) {
                JSONArray points = pathJSON.getJSONArray("sampled_points");
                for (int i = 0; i < points.length(); i++) {
                        JSONObject point = points.getJSONObject(i);
                        if (time <= point.getDouble("time")) {
                                return i;
                        }
                }
                return points.length() - 1;
        }

}