/* RobotModel/Mechs/Components/LimelightForSoda.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightForSoda extends MechComponent {

    private static class LimelightHelper {
        private Object networkTable;
        private boolean isConnected = false;

        public LimelightHelper(String tableName) {
            try {
                Class<?> ntClass = Class.forName("edu.wpi.first.networktables.NetworkTableInstance");
                Object instance = ntClass.getMethod("getDefault").invoke(null);
                this.networkTable = instance.getClass()
                        .getMethod("getTable", String.class)
                        .invoke(instance, tableName);
                this.isConnected = true;
            } catch (Exception e) {
                System.err.println("Limelight 3A connection failed: " + e.getMessage());
                this.isConnected = false;
            }
        }

        public boolean isConnected() {
            return isConnected && networkTable != null;
        }

        public double getDouble(String key, double defaultValue) {
            if (!isConnected) return defaultValue;
            try {
                Object entry = networkTable.getClass()
                        .getMethod("getEntry", String.class)
                        .invoke(networkTable, key);
                Object value = entry.getClass()
                        .getMethod("getDouble", double.class)
                        .invoke(entry, defaultValue);
                return (double) value;
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public double[] getDoubleArray(String key, double[] defaultValue) {
            if (!isConnected) return defaultValue;
            try {
                Object entry = networkTable.getClass()
                        .getMethod("getEntry", String.class)
                        .invoke(networkTable, key);
                Object value = entry.getClass()
                        .getMethod("getDoubleArray", double[].class)
                        .invoke(entry, defaultValue);
                return (double[]) value;
            } catch (Exception e) {
                return defaultValue;
            }
        }

        public void setNumber(String key, double value) {
            if (!isConnected) return;
            try {
                Object entry = networkTable.getClass()
                        .getMethod("getEntry", String.class)
                        .invoke(networkTable, key);
                entry.getClass()
                        .getMethod("setNumber", Number.class)
                        .invoke(entry, value);
            } catch (Exception e) {
                System.err.println("Failed to set Limelight value: " + e.getMessage());
            }
        }
    }

    public class AutonomousLimelightForSodaBehaviors extends AutonomousComponentBehaviors {

        /**
         * Check if Limelight 3A has a valid AprilTag in view
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
            return (int) limelightHelper.getDouble("tid", -1.0);
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
         * Robot's rotation/heading on field (0-360 degrees)
         */
        public double getRobotPoseRotation() {
            double[] botpose = limelightHelper.getDoubleArray("botpose", new double[6]);
            return botpose.length > 5 ? botpose[5] : 0.0;
        }

        /**
         * Distance from camera to AprilTag center
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
     * @param limelightName NetworkTables name (default: "limelight")
     * @param strategy Control strategy for gamepad input
     */
    public LimelightForSoda(HardwareMap hardwareMap,
                            String limelightName,
                            LimelightForSodaControlStrategy strategy) {
        super(strategy);
        this.strategy = strategy;
        this.limelightName = limelightName;
        this.limelightHelper = new LimelightHelper(limelightName);

        // Set to AprilTag detection pipeline (Pipeline 0)
        limelightHelper.setNumber("pipeline", 0);

        // Optional: Set LED mode to on
        limelightHelper.setNumber("ledMode", 0); // 0 = use pipeline setting
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
        telemetry.addData("Network Name", limelightName);
        telemetry.addData("Connected", limelightHelper.isConnected());

        if (!limelightHelper.isConnected()) {
            telemetry.addData("Status", "⚠ NOT CONNECTED");
            telemetry.addData("Check", "Ethernet/USB connection");
            return;
        }

        if (auton.hasAprilTagTarget()) {
            telemetry.addData("Target Found", "✓ YES");
            telemetry.addData("AprilTag ID", auton.getAprilTagId());
            telemetry.addData("Bearing (tx)", String.format("%.2f°", auton.getTargetBearing()));
            telemetry.addData("Elevation (ty)", String.format("%.2f°", auton.getTargetElevation()));
            telemetry.addData("Target Area", String.format("%.1f%%", auton.getTargetArea()));
            telemetry.addData("Distance", String.format("%.2f in", auton.getDistanceToTarget()));
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
