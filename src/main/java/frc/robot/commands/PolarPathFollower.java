package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.tools.wrappers.AutoFollower;

public class PolarPathFollower extends ParallelCommandGroup {
        private final JSONObject path;
        private final AutoFollower autoFollower;
        private volatile Double pathStartWallTimestamp = null;
        private volatile Double pathFinishWallTimestamp = null;
        private volatile double finalPathTime = -1.0;

        public PolarPathFollower(JSONObject path,
                        HashMap<String, Supplier<Command>> commandMap,
                        HashMap<String, BooleanSupplier> conditionMap) {
                this.path = path;
                this.autoFollower = new DoNothingFollower(path.optJSONArray("sampled_points"));
                this.finalPathTime = computeFinalPathTime();

                StartEndCommand pathTimerCommand = new StartEndCommand(
                                () -> {
                                        pathStartWallTimestamp = Timer.getFPGATimestamp();
                                        pathFinishWallTimestamp = null;
                                        Logger.recordOutput("Path Time Start", pathStartWallTimestamp);
                                },
                                () -> {
                                        pathFinishWallTimestamp = Timer.getFPGATimestamp();
                                        finalPathTime = Math.max(finalPathTime, computeFinalPathTime());
                                        pathStartWallTimestamp = null;
                                        Logger.recordOutput("Path Time Finish", pathFinishWallTimestamp);
                                });

                ArrayList<Command> pdgChildren = new ArrayList<>();
                pdgChildren.add(autoFollower);
                pdgChildren.add(pathTimerCommand);

                if (path.has("commands")) {
                        JSONArray cmds = path.getJSONArray("commands");
                        for (int i = 0; i < cmds.length(); i++) {
                                JSONObject cmdJson = cmds.getJSONObject(i);
                                Command parsed = runCommand(cmdJson, commandMap, conditionMap);
                                if (parsed != null)
                                        pdgChildren.add(parsed);

                        }
                }

                ParallelDeadlineGroup pdg = new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> autoFollower.isFinished()),
                                pdgChildren.toArray(new Command[0]));

                addCommands(pdg);
        }

        private Command runCommand(JSONObject command,
                        HashMap<String, Supplier<Command>> commandMap,
                        HashMap<String, BooleanSupplier> conditionMap) {
                double start = command.optDouble("start", 0.0);
                double end = command.optDouble("end", Double.MAX_VALUE);

                BooleanSupplier Start = () -> GetTime() >= start;
                BooleanSupplier End = () -> GetTime() >= end;

                if (command.has("command")) {
                        JSONObject cobj = command.getJSONObject("command");
                        String name = cobj.getString("name");
                        Supplier<Command> sup = commandMap.get(name);
                        if (sup == null) {
                                return null;
                        }
                        StartEndCommand deferred = new StartEndCommand(
                                        () -> CommandScheduler.getInstance().schedule(sup.get()),
                                        () -> {
                                        });
                        return new TriggerCommand(Start, deferred, End);
                } else if (command.has("branched_command")) {
                        JSONObject branched = command.getJSONObject("branched_command");
                        String condName = branched.optString("condition", null);
                        BooleanSupplier condition = (conditionMap != null && condName != null)
                                        ? conditionMap.get(condName)
                                        : () -> false;
                        if (condName != null && (condition == null)) {

                                condition = () -> false;
                        }
                        JSONObject onTrue = branched.getJSONObject("on_true");
                        JSONObject onFalse = branched.getJSONObject("on_false");
                        Command trueCmd = runCommand(onTrue, commandMap, conditionMap);
                        Command falseCmd = runCommand(onFalse, commandMap, conditionMap);
                        return new TriggerCommand(Start, new ConditionalCommand(trueCmd, falseCmd, condition), End);
                } else if (command.has("parallel_command_group")) {
                        JSONArray arr = command.getJSONObject("parallel_command_group").getJSONArray("commands");
                        ArrayList<Command> subs = new ArrayList<>();
                        for (int i = 0; i < arr.length(); i++) {
                                Command sub = runCommand(arr.getJSONObject(i), commandMap, conditionMap);
                                if (sub != null)
                                        subs.add(sub);
                        }
                        return new TriggerCommand(Start, new ParallelCommandGroup(subs.toArray(new Command[0])), End);
                } else if (command.has("sequential_command_group")) {
                        JSONArray arr = command.getJSONObject("sequential_command_group").getJSONArray("commands");
                        ArrayList<Command> subs = new ArrayList<>();
                        for (int i = 0; i < arr.length(); i++) {
                                Command sub = runCommand(arr.getJSONObject(i), commandMap, conditionMap);
                                if (sub != null)
                                        subs.add(sub);
                        }
                        return new TriggerCommand(Start, new SequentialCommandGroup(subs.toArray(new Command[0])), End);
                } else if (command.has("parallel_deadline_group")) {
                        JSONObject pdg = command.getJSONObject("parallel_deadline_group");
                        Command deadline = runCommand(pdg.getJSONArray("deadline_command").getJSONObject(0), commandMap,
                                        conditionMap);
                        JSONArray arr = pdg.getJSONArray("commands");
                        ArrayList<Command> subs = new ArrayList<>();
                        for (int i = 0; i < arr.length(); i++) {
                                Command sub = runCommand(arr.getJSONObject(i), commandMap, conditionMap);
                                if (sub != null)
                                        subs.add(sub);
                        }
                        return new TriggerCommand(Start,
                                        new ParallelDeadlineGroup(deadline, subs.toArray(new Command[0])), End);
                } else if (command.has("parallel_race_group")) {
                        JSONArray arr = command.getJSONObject("parallel_race_group").getJSONArray("commands");
                        ArrayList<Command> subs = new ArrayList<>();
                        for (int i = 0; i < arr.length(); i++) {
                                Command sub = runCommand(arr.getJSONObject(i), commandMap, conditionMap);
                                if (sub != null)
                                        subs.add(sub);
                        }
                        return new TriggerCommand(Start, new ParallelRaceGroup(subs.toArray(new Command[0])), End);
                }

                throw new IllegalArgumentException("PolarPathFollower: Invalid command JSON: " + command.toString());
        }

        private double GetTime() {
                double now = Timer.getFPGATimestamp();

                if (pathStartWallTimestamp != null) {
                        double elapsed = now - pathStartWallTimestamp;
                        if (elapsed < 0.0)
                                elapsed = 0.0;
                        if (finalPathTime >= 0.0 && elapsed > finalPathTime)
                                elapsed = finalPathTime;
                        Logger.recordOutput("Path Time", elapsed);
                        return elapsed;
                }

                if (pathFinishWallTimestamp != null) {
                        double after = now - pathFinishWallTimestamp;
                        if (after < 0.0)
                                after = 0.0;
                        double t = (finalPathTime >= 0.0) ? finalPathTime : 0.0;
                        double result = t + after;
                        Logger.recordOutput("Path Time", result);
                        return result;
                }

                Logger.recordOutput("Path Time", 0.0);
                return 0.0;
        }

        private double computeFinalPathTime() {
                double maxT = -1.0;

                if (path.has("sampled_points")) {
                        JSONArray sp = path.getJSONArray("sampled_points");
                        if (sp.length() > 0) {
                                double t = sp.getJSONObject(sp.length() - 1).optDouble("time", -1.0);
                                if (t > maxT)
                                        maxT = t;
                        }
                }

                if (path.has("key_points")) {
                        JSONArray kp = path.getJSONArray("key_points");
                        if (kp.length() > 0) {
                                double t = kp.getJSONObject(kp.length() - 1).optDouble("time", -1.0);
                                if (t > maxT)
                                        maxT = t;
                        }
                }

                if (path.has("time")) {
                        double t = path.optDouble("time", -1.0);
                        if (t > maxT)
                                maxT = t;
                }

                return maxT;
        }

        private double getMaxPathTime() {
                if (finalPathTime < 0.0)
                        finalPathTime = computeFinalPathTime();
                return finalPathTime;
        }
}
