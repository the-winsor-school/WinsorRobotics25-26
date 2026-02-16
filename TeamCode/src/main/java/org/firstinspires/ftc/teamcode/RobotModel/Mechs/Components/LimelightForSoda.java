/* RobotModel/Mechs/Components/LimelightForSoda.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Limelight replacement using FTC's built-in AprilTag vision
 * This uses the standard FTC SDK without external NetworkTables dependency
 */
public class LimelightForSoda extends MechComponent {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final String cameraName;
    private boolean isConnected = false;

    public class AutonomousLimelightForSodaBehaviors extends AutonomousComponentBehaviors {

        /**
         * Check if we have a valid AprilTag target
         * @return true if at least one AprilTag is detected
         */
        public boolean hasAprilTagTarget() {
            if (!isConnected) return false;
            try {
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                return detections != null && detections.size() > 0;
            } catch (Exception e) {
                return false;
            }
        }

        /**
         * Get horizontal offset to first detected tag (in degrees)
         * Negative = left, Positive = right
         * @return tx value in degrees
         */
        public double getTargetX() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.bearing;  // Horizontal angle
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Get vertical offset to first detected tag (in degrees)
         * Negative = below, Positive = above
         * @return ty value in degrees
         */
        public double getTargetY() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.elevation;  // Vertical angle
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Get distance to first detected tag (in inches)
         * @return range in inches
         */
        public double getTargetArea() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.range;  // Distance in inches
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Get detected AprilTag ID
         * @return tag ID, or -1 if none
         */
        public int getAprilTagId() {
            if (!isConnected || !hasAprilTagTarget()) return -1;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null) {
                    return tag.id;
                }
            } catch (Exception e) {
                // Silently fail
            }
            return -1;
        }

        /**
         * Get robot's X position on field
         * @return x position in inches
         */
        public double getRobotPoseX() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.x;
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Get robot's Y position on field
         * @return y position in inches
         */
        public double getRobotPoseY() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.y;
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Get robot's rotation on field (in degrees)
         * @return heading in degrees
         */
        public double getRobotPoseRotation() {
            if (!isConnected || !hasAprilTagTarget()) return 0.0;
            try {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                if (tag != null && tag.ftcPose != null) {
                    return tag.ftcPose.yaw;  // Heading/yaw
                }
            } catch (Exception e) {
                // Silently fail
            }
            return 0.0;
        }

        /**
         * Check if vision system is connected
         */
        public boolean isLimelightConnected() {
            return isConnected;
        }

        /**
         * Get all detected AprilTags
         */
        public List<AprilTagDetection> getAllDetections() {
            if (!isConnected) return null;
            try {
                return aprilTagProcessor.getDetections();
            } catch (Exception e) {
                return null;
            }
        }

        /**
         * Find a specific AprilTag by ID
         */
        public AprilTagDetection findTagById(int tagId) {
            if (!isConnected) return null;
            try {
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                if (detections == null) return null;
                for (AprilTagDetection tag : detections) {
                    if (tag.id == tagId) {
                        return tag;
                    }
                }
            } catch (Exception e) {
                // Silently fail
            }
            return null;
        }
    }

    public interface LimelightForSodaControlStrategy extends IControlStrategy {
        void processLimelightData(Gamepad gamepad);
    }

    private final LimelightForSodaControlStrategy strategy;
    private final AutonomousLimelightForSodaBehaviors auton;

    /**
     * Initialize vision system with AprilTag detection
     * @param hardwareMap HardwareMap from OpMode
     * @param cameraName Name of camera in hardware config
     * @param strategy Control strategy for teleop
     */
    public LimelightForSoda(HardwareMap hardwareMap,
                            String cameraName,
                            LimelightForSodaControlStrategy strategy) {
        super(strategy);
        this.cameraName = cameraName;
        this.strategy = strategy;

        try {
            // Create AprilTag processor
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

            // Create vision portal with AprilTag processor
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, cameraName))
                    .addProcessor(aprilTagProcessor)
                    .build();

            isConnected = true;
        } catch (Exception e) {
            aprilTagProcessor = null;
            visionPortal = null;
            isConnected = false;
        }

        this.auton = new AutonomousLimelightForSodaBehaviors();
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
            telemetry.addData("Vision System", cameraName + " - NOT CONNECTED");
            telemetry.addLine("Check camera configuration");
            return;
        }

        boolean hasTarget = auton.hasAprilTagTarget();
        telemetry.addData("Vision System", cameraName + " - Connected");
        telemetry.addData("AprilTags Detected", hasTarget);

        if (hasTarget) {
            AprilTagDetection firstTag = aprilTagProcessor.getDetections().get(0);
            telemetry.addData("First Tag ID", firstTag.id);
            if (firstTag.ftcPose != null) {
                telemetry.addData("Bearing (X degrees)", String.format("%.2f", auton.getTargetX()));
                telemetry.addData("Elevation (Y degrees)", String.format("%.2f", auton.getTargetY()));
                telemetry.addData("Range (inches)", String.format("%.2f", auton.getTargetArea()));
                telemetry.addData("Yaw (rotation)", String.format("%.2f°", auton.getRobotPoseRotation()));
            }
        } else {
            telemetry.addData("Status", "No AprilTag in view");
        }
    }

    /**
     * Close the vision portal when done
     */
    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
