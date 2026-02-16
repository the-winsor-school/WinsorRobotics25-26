/* RobotModel/Mechs/Components/LimelightForSoda.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

import org.json.JSONObject;
import org.json.JSONArray;

public class LimelightForSoda extends MechComponent {

    private static class LimelightHelper {
        private final String limelightIP;
        private boolean isConnected = false;
        private long lastUpdateTime = 0;
        private JSONObject lastResponse = null;
        private static final long UPDATE_INTERVAL = 50; // ms between queries

        public LimelightHelper(String limelightIP) {
            this.limelightIP = limelightIP;
            testConnection();
        }

        /**
         * Test if Limelight is reachable
         */
        private void testConnection() {
            new Thread(() -> {
                try {
                    URL url = new URL("http://" + limelightIP + ":5800/");
                    HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                    connection.setConnectTimeout(2000);
                    connection.setReadTimeout(2000);
                    int responseCode = connection.getResponseCode();
                    isConnected = (responseCode == 200);
                    connection.disconnect();
                } catch (Exception e) {
                    isConnected = false;
                }
            }).start();
        }

        public boolean isConnected() {
            return isConnected;
        }

        /**
         * Query Limelight JSON endpoint
         * Returns: {"Results":{"Fiducial":{"Ids":[...], "tx":..., "ty":..., ...}}}
         */
        private JSONObject queryLimelight() {
            // Rate limit queries to avoid overwhelming the network
            long now = System.currentTimeMillis();
            if (now - lastUpdateTime < UPDATE_INTERVAL) {
                return lastResponse;
            }

            try {
                URL url = new URL("http://" + limelightIP + ":5800/limelight/results");
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(1000);
                connection.setReadTimeout(1000);

                BufferedReader reader = new BufferedReader(
                        new InputStreamReader(connection.getInputStream()));
                StringBuilder response = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    response.append(line);
                }
                reader.close();
                connection.disconnect();

                lastResponse = new JSONObject(response.toString());
                lastUpdateTime = now;
                isConnected = true;
                return lastResponse;

            } catch (Exception e) {
                System.err.println("Limelight query failed: " + e.getMessage());
                isConnected = false;
                return null;
            }
        }

        public double getDouble(String key, double defaultValue) {
            try {
                JSONObject results = queryLimelight();
                if (results == null) return defaultValue;

                JSONObject fiducial = results.optJSONObject("Results")
                        .optJSONObject("Fiducial");
                if (fiducial == null) return defaultValue;

                return fiducial.optDouble(key, defaultValue);
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public int getInt(String key, int defaultValue) {
            try {
                JSONObject results = queryLimelight();
                if (results == null) return defaultValue;

                JSONObject fiducial = results.optJSONObject("Results")
                        .optJSONObject("Fiducial");
                if (fiducial == null) return defaultValue;

                return fiducial.optInt(key, defaultValue);
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public double[] getDoubleArray(String key, double[] defaultValue) {
            try {
                JSONObject results = queryLimelight();
                if (results == null) return defaultValue;

                JSONObject fiducial = results.optJSONObject("Results")
                        .optJSONObject("Fiducial");
                if (fiducial == null) return defaultValue;

                JSONArray array = fiducial.optJSONArray(key);
                if (array == null) return defaultValue;

                double[] result = new double[array.length()];
                for (int i = 0; i < array.length(); i++) {
                    result[i] = array.getDouble(i);
                }
                return result;
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public int[] getIntArray(String key, int[] defaultValue) {
            try {
                JSONObject results = queryLimelight();
                if (results == null) return defaultValue;

                JSONObject fiducial = results.optJSONObject("Results")
                        .optJSONObject("Fiducial");
                if (fiducial == null) return defaultValue;

                JSONArray array = fiducial.optJSONArray(key);
                if (array == null) return defaultValue;

                int[] result = new int[array.length()];
                for (int i = 0; i < array.length(); i++) {
                    result[i] = array.getInt(i);
                }
                return result;
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public void setNumber(String key, double value) {
            new Thread(() -> {
                try {
                    URL url = new URL("http://" + limelightIP + ":5800/limelight/settings?key="
                            + key + "&value=" + value);
                    HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                    connection.setConnectTimeout(500);
                    connection.setReadTimeout(500);
                    connection.getResponseCode();
                    connection.disconnect();
                } catch (Exception e) {
                    System.err.println("Failed to set Limelight value: " + e.getMessage());
                }
            }).start();
        }
    }

    public class AutonomousLimelightForSodaBehaviors extends AutonomousComponentBehaviors {

        /**
         * Check if Limelight has a valid AprilTag in view
         * tv = 1 when target is detected
         */
        public boolean hasAprilTagTarget() {
            return limelightHelper.getDouble("tv", 0) == 1.0;
        }

        /**
         * Horizontal offset to target (-27 to 27 degrees)
         * Positive = target is to the right
         */
        public double getTargetX() {
            return limelightHelper.getDouble("tx", 0.0);
        }

        /**
         * Vertical offset to target (-20.5 to 20.5 degrees)
         * Positive = target is above crosshair
         */
        public double getTargetY() {
            return limelightHelper.getDouble("ty", 0.0);
        }

        /**
         * Target area as percentage of image (0-100)
         */
        public double getTargetArea() {
            return limelightHelper.getDouble("ta", 0.0);
        }

        /**
         * AprilTag ID detected (0-587 for 36h11 family)
         */
        public int getAprilTagId() {
            return limelightHelper.getInt("tid", -1);
        }

        /**
         * All AprilTag IDs currently in view
         */
        public int[] getAllAprilTagIds() {
            return limelightHelper.getIntArray("tids", new int[0]);
        }

        /**
         * Robot's X position on field (from AprilTag pose)
         * botpose = [x, y, z, roll, pitch, yaw]
         */
        public double getRobotPoseX() {
            double[] botpose = limelightHelper.getDoubleArray("botpose", new double[6]);
            return botpose.length > 0 ? botpose[0] : 0.0;
        }

        /**
         * Robot's Y position on field
         */
        public double getRobotPoseY() {
            double[] botpose = limelightHelper.getDoubleArray("botpose", new double[6]);
            return botpose.length > 1 ? botpose[1] : 0.0;
        }

        /**
         * Robot's Z position on field
         */
        public double getRobotPoseZ() {
            double[] botpose = limelightHelper.getDoubleArray("botpose", new double[6]);
            return botpose.length > 2 ? botpose[2] : 0.0;
        }

        /**
         * Robot's rotation/heading on field (0-360 degrees)
         */
        public double getRobotPoseRotation() {
            double[] botpose = limelightHelper.getDoubleArray("botpose", new double[6]);
            return botpose.length > 5 ? botpose[5] : 0.0;
        }

        /**
         * Distance from camera to AprilTag center (inches)
         */
        public double getDistanceToTarget() {
            if (hasAprilTagTarget()) {
                double[] botposeTargetSpace = limelightHelper.getDoubleArray("botpose_targetspace", new double[6]);
                if (botposeTargetSpace.length > 1) {
                    return Math.hypot(botposeTargetSpace[0], botposeTargetSpace[1]);
                }
            }
            return 0.0;
        }

        /**
         * Horizontal bearing to target (same as tx)
         */
        public double getTargetBearing() {
            return getTargetX();
        }

        /**
         * Vertical elevation to target (same as ty)
         */
        public double getTargetElevation() {
            return getTargetY();
        }

        /**
         * Latency from Limelight in milliseconds
         */
        public double getLatency() {
            return limelightHelper.getDouble("tl", 0.0);
        }
    }

    public interface LimelightForSodaControlStrategy extends IControlStrategy {
        void processLimelightData(Gamepad gamepad);
    }

    private final LimelightHelper limelightHelper;
    private final LimelightForSodaControlStrategy strategy;
    private final AutonomousLimelightForSodaBehaviors auton = new AutonomousLimelightForSodaBehaviors();
    private final String limelightName;

    /**
     * Initialize Limelight 3A component
     * @param hardwareMap HardwareMap from OpMode
     * @param limelightIP IP address of Limelight (e.g., "192.168.1.100" or "limelight.local")
     * @param strategy Control strategy for gamepad input
     */
    public LimelightForSoda(HardwareMap hardwareMap,
                            String limelightIP,
                            LimelightForSodaControlStrategy strategy) {
        super(strategy);
        this.strategy = strategy;
        this.limelightName = limelightIP;
        this.limelightHelper = new LimelightHelper(limelightIP);
    }

    @Override
    public AutonomousLimelightForSodaBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.processLimelightData(gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addLine("=== Limelight 3A ===");
        telemetry.addData("IP Address", limelightName);
        telemetry.addData("Connected", limelightHelper.isConnected());

        if (!limelightHelper.isConnected()) {
            telemetry.addData("Status", "⚠ NOT CONNECTED");
            telemetry.addData("Check", "Network connection / IP address");
            telemetry.addData("Tip", "Use limelight.local or find IP in web UI");
            return;
        }

        if (auton.hasAprilTagTarget()) {
            telemetry.addData("Target Found", "✓ YES");
            telemetry.addData("AprilTag ID", auton.getAprilTagId());
            telemetry.addData("Bearing (tx)", String.format("%.2f°", auton.getTargetBearing()));
            telemetry.addData("Elevation (ty)", String.format("%.2f°", auton.getTargetElevation()));
            telemetry.addData("Target Area", String.format("%.1f%%", auton.getTargetArea()));
            telemetry.addData("Distance", String.format("%.2f in", auton.getDistanceToTarget()));
            telemetry.addData("Latency", String.format("%.1f ms", auton.getLatency()));
            telemetry.addLine();
            telemetry.addData("Robot X", String.format("%.2f in", auton.getRobotPoseX()));
            telemetry.addData("Robot Y", String.format("%.2f in", auton.getRobotPoseY()));
            telemetry.addData("Robot Heading", String.format("%.2f°", auton.getRobotPoseRotation()));
        } else {
            telemetry.addData("Target Found", "✗ NO");
            telemetry.addData("Status", "Searching for AprilTag...");
        }
    }
}
