package frc.robot.tools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.Filesystem;

public class PathLoader {

    public static class PosePoint {
        public final double x, y, theta, time;

        public PosePoint(double x, double y, double theta, double time) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.time = time;
        }
    }

    public static List<PosePoint> loadPath(String relativePath) throws IOException {
        File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
        String content = Files.readString(file.toPath());
        System.out.println(content);
        JSONObject root = new JSONObject(content);
        JSONArray arr = root.getJSONArray("sampled_points");
        List<PosePoint> points = new ArrayList<>();
        for (int i = 0; i < arr.length(); i++) {
            JSONObject p = arr.getJSONObject(i);
            points.add(new PosePoint(
                    p.getDouble("x"),
                    p.getDouble("y"),
                    p.getDouble("angle"),
                    p.getDouble("time")));
        }
        return points;
    }
}
