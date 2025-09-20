// package frc.robot.tools;

// import java.io.File;
// import java.io.IOException;
// import java.nio.file.Files;
// import java.util.ArrayList;
// import java.util.List;

// import org.json.JSONArray;
// import org.json.JSONObject;

// import edu.wpi.first.wpilibj.Filesystem;

// public class PathLoader {

//     public static class PosePoint {
//         public final double x, y, theta, time;
//         public final double dx, dy, dtheta;

//         public PosePoint(double x, double y, double theta, double time,
//                          double dx, double dy, double dtheta) {
//             this.x = x;
//             this.y = y;
//             this.theta = theta;
//             this.time = time;
//             this.dx = dx;
//             this.dy = dy;
//             this.dtheta = dtheta;
//         }
//     }

//     public static List<PosePoint> loadPath(String relativePath) throws IOException {
//         File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
//         String content = Files.readString(file.toPath());
//         JSONObject root = new JSONObject(content);
//         JSONArray arr = root.getJSONArray("sampled_points");
//         List<PosePoint> points = new ArrayList<>();
//         for (int i = 0; i < arr.length(); i++) {
//             JSONObject p = arr.getJSONObject(i);
//             points.add(new PosePoint(
//                     p.getDouble("x"),
//                     p.getDouble("y"),
//                     p.getDouble("angle"),
//                     p.getDouble("time"),
//                     p.getDouble("x_velocity"),
//                     p.getDouble("y_velocity"),
//                     p.getDouble("angular_velocity")));
//         }
//         return points;
//     }

//     public static JSONObject getJSON(String relativePath) throws IOException {
//         File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
//         String content = Files.readString(file.toPath());
//         JSONObject root = new JSONObject(content);
//         return root;
//     }
// }
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
        public final double dx, dy, dtheta;

        public PosePoint(double x, double y, double theta, double time,
                         double dx, double dy, double dtheta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.time = time;
            this.dx = dx;
            this.dy = dy;
            this.dtheta = dtheta;
        }
    }

    public static List<PosePoint> loadPath(String relativePath) throws IOException {
        File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
        String content = Files.readString(file.toPath());
        JSONObject root = new JSONObject(content);
        JSONArray arr = root.getJSONArray("sampled_points");
        List<PosePoint> points = new ArrayList<>();
        for (int i = 0; i < arr.length(); i++) {
            JSONObject p = arr.getJSONObject(i);
            points.add(new PosePoint(
                    p.getDouble("x"),
                    p.getDouble("y"),
                    p.getDouble("angle"),
                    p.getDouble("time"),
                    p.getDouble("x_velocity"),
                    p.getDouble("y_velocity"),
                    p.getDouble("angular_velocity")));
        }
        return points;
    }
    
    public static List<PosePoint> loadAuto(String relativePath) throws IOException {
        File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
        String content = Files.readString(file.toPath());
        JSONObject root = new JSONObject(content);
        JSONArray paths = root.getJSONArray("paths");
        List<PosePoint> points = new ArrayList<>();
        for (int j = 0; j < paths.length(); j++) {
            JSONObject path = paths.getJSONObject(j);
            JSONArray arr = path.getJSONArray("sampled_points");
            for (int i = 0; i < arr.length(); i++) {
                JSONObject p = arr.getJSONObject(i);
                points.add(new PosePoint(
                        p.getDouble("x"),
                        p.getDouble("y"),
                        p.getDouble("angle"),
                        p.getDouble("time"),
                        p.getDouble("x_velocity"),
                        p.getDouble("y_velocity"),
                        p.getDouble("angular_velocity")));
            }
        }
        return points;
    }

    public static JSONObject getJSON(String relativePath) throws IOException {
        File file = new File(Filesystem.getDeployDirectory() + "/" + "Paths/" + relativePath);
        String content = Files.readString(file.toPath());
        JSONObject root = new JSONObject(content);
        return root;
    }
}
