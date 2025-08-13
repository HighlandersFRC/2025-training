package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PolarAutoFollower extends SequentialCommandGroup {
    public PolarAutoFollower(JSONObject auto,
            HashMap<String, Supplier<Command>> commandMap,
            HashMap<String, BooleanSupplier> conditionMap) {

        System.out.println("PolarAutoFollower: constructor entered");

        JSONArray schedule = auto.optJSONArray("schedule");
        JSONArray paths = auto.optJSONArray("paths");

        if (schedule == null || paths == null) {
            throw new IllegalArgumentException("PolarAutoFollower: invalid auto JSON (missing schedule or paths)");
        }

        for (int i = 0; i < schedule.length(); i++) {
            System.out.println("PolarAutoFollower: loop " + i);

            JSONObject step = schedule.getJSONObject(i);
            boolean branched = step.optBoolean("branched", false);

            if (!branched) {
                int pathIdx = step.getInt("path");
                if (pathIdx < 0 || pathIdx >= paths.length()) {
                    throw new IndexOutOfBoundsException("PolarAutoFollower: invalid path index " + pathIdx);
                }
                addCommands(new PolarPathFollower(paths.getJSONObject(pathIdx), commandMap, conditionMap));
            } else {
                String condName = step.optString("condition", null);
                BooleanSupplier condition = (conditionMap != null && condName != null) ? conditionMap.get(condName)
                        : () -> false;

                JSONArray onTrue = step.optJSONArray("on_true");
                JSONArray onFalse = step.optJSONArray("on_false");
                if (onTrue == null || onFalse == null) {
                    throw new IllegalArgumentException(
                            "PolarAutoFollower: branched step missing on_true/on_false arrays");
                }

                JSONObject onTrueObj = new JSONObject();
                JSONObject onFalseObj = new JSONObject();
                onTrueObj.put("paths", paths);
                onTrueObj.put("schedule", onTrue);
                onFalseObj.put("paths", paths);
                onFalseObj.put("schedule", onFalse);

                PolarAutoFollower tf = new PolarAutoFollower(onTrueObj, commandMap, conditionMap);
                PolarAutoFollower ff = new PolarAutoFollower(onFalseObj, commandMap, conditionMap);

                addCommands(new ConditionalCommand(tf, ff, condition));
            }
        }

    }
}
