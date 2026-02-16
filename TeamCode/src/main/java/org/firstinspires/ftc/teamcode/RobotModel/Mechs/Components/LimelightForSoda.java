/* RobotModel/Mechs/Components/LimelightForSoda.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Limelight integration for Sodapop
 * Connects to Limelight via NetworkTables for AprilTag detection and tracking
 *
 * SETUP REQUIRED:
 * 1. Limelight must be on same network as Control Hub/RC phone
 * 2. Limelight IP: http://limelight.local:5801 (default)
 * 3. Set pipeline to AprilTag detection in Limelight web interface
 */
public class LimelightForSoda extends MechComponent {

    private final String limelightName;
    private boolean isConnected = false;

    // NetworkTables objects (using reflection to avoid import errors)
    private Object networkTableInstance;
    private Object limelightTable;

    public class AutonomousLimelightForSodaBehaviors extends AutonomousComponentBehaviors {

        /**
         * Check if Limelight has a valid AprilTag target
         * @return true if tv (valid target) == 1
         */
        public boolean hasAprilTagTarget() {
            if (!isConnected) return false;
            try {
                return getNetworkTableDouble("tv", 0) == 1.0;
            } catch (Exception e) {
                return false;
            }
        }

        /**
         * Get horizontal offset from crosshair to target (-27 to 27 degrees)
         * Negative = target is to the left, Positive = target is to the right
         * @return tx value in degrees
         */
        public double getTargetX() {
            if (!isConnected) return 0.0;
            try {
                return getNetworkTableDouble("tx", 0.0);
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Get vertical offset from crosshair to target (-20.5 to 20.5 degrees)
         * Negative = target is below, Positive = target is above
         * @return ty value in degrees
         */
        public double getTargetY() {
            if (!isConnected) return 0.0;
            try {
                return getNetworkTableDouble("ty", 0.0);
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Get target area as percentage of image (0-100)
         * Larger area = closer to target
         * @return ta value
         */
        public double getTargetArea() {
            if (!isConnected) return 0.0;
            try {
                return getNetworkTableDouble("ta", 0.0);
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Get detected AprilTag ID
         * @return tid (tag ID), or -1 if none
         */
        public int getAprilTagId() {
            if (!isConnected) return -1;
            try {
                return (int) getNetworkTableDouble("tid", -1);
            } catch (Exception e) {
                return -1;
            }
        }

        /**
         * Get robot's X position on field (requires AprilTag calibration)
         * @return botpose[0]
         */
        public double getRobotPoseX() {
            if (!isConnected) return 0.0;
            try {
                double[] botpose = getNetworkTableDoubleArray("botpose", new double[6]);
                return botpose.length > 0 ? botpose[0] : 0.0;
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Get robot's Y position on field
         * @return botpose[1]
         */
        public double getRobotPoseY() {
            if (!isConnected) return 0.0;
            try {
                double[] botpose = getNetworkTableDoubleArray("botpose", new double[6]);
                return botpose.length > 1 ? botpose[1] : 0.0;
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Get robot's rotation on field (0-360 degrees)
         * @return botpose[5]
         */
        public double getRobotPoseRotation() {
            if (!isConnected) return 0.0;
            try {
                double[] botpose = getNetworkTableDoubleArray("botpose", new double[6]);
                return botpose.length > 5 ? botpose[5] : 0.0;
            } catch (Exception e) {
                return 0.0;
            }
        }

        /**
         * Set Limelight LED mode
         * @param mode 0=pipeline default, 1=off, 2=blink, 3=on
         */
        public void setLEDMode(int mode) {
            if (!isConnected) return;
            try {
                setNetworkTableNumber("ledMode", mode);
            } catch (Exception e) {
                // Silently fail
            }
        }

        /**
         * Set Limelight pipeline (0-9)
         * @param pipeline pipeline index
         */
        public void setPipeline(int pipeline) {
            if (!isConnected) return;
            try {
                setNetworkTableNumber("pipeline", pipeline);
            } catch (Exception e) {
                // Silently fail
            }
        }

        /**
         * Check if Limelight is connected
         */
        public boolean isLimelightConnected() {
            return isConnected;
        }
    }

    public interface LimelightForSodaControlStrategy extends IControlStrategy {
        void processLimelightData(Gamepad gamepad);
    }

    private final LimelightForSodaControlStrategy strategy;
    private final AutonomousLimelightForSodaBehaviors auton;

    /**
     * Initialize Limelight component with NetworkTables
     * @param hardwareMap HardwareMap from OpMode
     * @param limelightName Name of Limelight in NetworkTables (typically "limelight")
     * @param strategy Control strategy for teleop
     */
    public LimelightForSoda(HardwareMap hardwareMap,
                            String limelightName,
                            LimelightForSodaControlStrategy strategy) {
        super(strategy);
        this.limelightName = limelightName;
        this.strategy = strategy;

        // Initialize NetworkTables connection to Limelight
        initializeNetworkTables(limelightName);

        this.auton = new AutonomousLimelightForSodaBehaviors();
    }

    /**
     * Initialize NetworkTables using reflection (avoids import issues)
     */
    private void initializeNetworkTables(String tableName) {
        try {
            // Get NetworkTableInstance using reflection
            Class<?> ntClass = Class.forName("edu.wpi.first.networktables.NetworkTableInstance");
            java.lang.reflect.Method getDefaultMethod = ntClass.getMethod("getDefault");
            networkTableInstance = getDefaultMethod.invoke(null);

            // Get the Limelight table
            java.lang.reflect.Method getTableMethod = ntClass.getMethod("getTable", String.class);
            limelightTable = getTableMethod.invoke(networkTableInstance, tableName);

            // Set LED mode to on
            setNetworkTableNumber("ledMode", 3);

            isConnected = true;
        } catch (Exception e) {
            isConnected = false;
            // Will fall back to mock mode
        }
    }

    /**
     * Get a double value from NetworkTables
     */
    private double getNetworkTableDouble(String key, double defaultValue) throws Exception {
        Class<?> tableClass = limelightTable.getClass();
        java.lang.reflect.Method getEntryMethod = tableClass.getMethod("getEntry", String.class);
        Object entry = getEntryMethod.invoke(limelightTable, key);

        Class<?> entryClass = entry.getClass();
        java.lang.reflect.Method getDoubleMethod = entryClass.getMethod("getDouble", double.class);
        return (double) getDoubleMethod.invoke(entry, defaultValue);
    }

    /**
     * Get a double array from NetworkTables
     */
    private double[] getNetworkTableDoubleArray(String key, double[] defaultValue) throws Exception {
        Class<?> tableClass = limelightTable.getClass();
        java.lang.reflect.Method getEntryMethod = tableClass.getMethod("getEntry", String.class);
        Object entry = getEntryMethod.invoke(limelightTable, key);

        Class<?> entryClass = entry.getClass();
        java.lang.reflect.Method getArrayMethod = entryClass.getMethod("getDoubleArray", double[].class);
        return (double[]) getArrayMethod.invoke(entry, (Object) defaultValue);
    }

    /**
     * Set a number value in NetworkTables
     */
    private void setNetworkTableNumber(String key, Number value) throws Exception {
        Class<?> tableClass = limelightTable.getClass();
        java.lang.reflect.Method getEntryMethod = tableClass.getMethod("getEntry", String.class);
        Object entry = getEntryMethod.invoke(limelightTable, key);

        Class<?> entryClass = entry.getClass();
        java.lang.reflect.Method setMethod = entryClass.getMethod("setNumber", Number.class);
        setMethod.invoke(entry, value);
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
        if (!isConnected) {
            telemetry.addData("Limelight", limelightName + " - NOT CONNECTED");
            telemetry.addLine("Check: 1) Limelight on network, 2) AprilTag pipeline enabled");
            telemetry.addLine("Limelight IP: http://limelight.local:5801");
            return;
        }

        boolean hasTarget = auton.hasAprilTagTarget();
        telemetry.addData("Limelight", limelightName + " - Connected");
        telemetry.addData("Has AprilTag Target", hasTarget);

        if (hasTarget) {
            telemetry.addData("Target X (degrees)", String.format("%.2f", auton.getTargetX()));
            telemetry.addData("Target Y (degrees)", String.format("%.2f", auton.getTargetY()));
            telemetry.addData("Target Distance (in)", String.format("%.2f", auton.getTargetArea()));
            telemetry.addData("AprilTag ID", auton.getAprilTagId());
            telemetry.addData("Robot Pose X", String.format("%.2f", auton.getRobotPoseX()));
            telemetry.addData("Robot Pose Y", String.format("%.2f", auton.getRobotPoseY()));
            telemetry.addData("Robot Rotation", String.format("%.2f°", auton.getRobotPoseRotation()));
        } else {
            telemetry.addData("Status", "No AprilTag in view - searching...");
        }
    }
}
