package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.tools.wrappers.AutoFollower;

public class PolarAutoFollower extends SequentialCommandGroup {
    AutoFollower autoFollower;

    public PolarAutoFollower(JSONObject auto, HashMap<String, Supplier<Command>> commandMap,
            HashMap<String, BooleanSupplier> conditionMap) {

        JSONArray schedule = auto.getJSONArray("schedule");
        JSONArray paths = auto.getJSONArray("paths");

        for (int i = 0; i < schedule.length(); i++) {
            if (!schedule.getJSONObject(i).getBoolean("branched")) {
                int pathIdx = schedule.getJSONObject(i).getInt("path");
                addCommands(new PolarPathFollower(paths.getJSONObject(pathIdx), commandMap, conditionMap));
            } else {
                BooleanSupplier condition = conditionMap.get(schedule.getJSONObject(i).getString("condition"));
                JSONArray OnTrue = schedule.getJSONObject(i).getJSONArray("on_true");
                JSONArray OnFalse = schedule.getJSONObject(i).getJSONArray("on_false");
                JSONObject onTrueObj = new JSONObject();
                JSONObject onFalseObj = new JSONObject();
                onTrueObj.put("paths", paths);
                onFalseObj.put("paths", paths);
                onFalseObj.put("schedule", OnFalse);
                onTrueObj.put("schedule", OnTrue);
                PolarAutoFollower onTrueFollower = new PolarAutoFollower(onTrueObj, commandMap, conditionMap);
                PolarAutoFollower onFalseFollower = new PolarAutoFollower(onFalseObj, commandMap, conditionMap);
                addCommands(new ConditionalCommand(onTrueFollower, onFalseFollower, condition));
                if (commandMap.containsKey("")) {

                }

            }

        }
    }
}
