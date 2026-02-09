/* RobotModel/Mechs/Components/LimelightVision.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LimelightVision extends MechComponent {

    public class AutonomousLimelightBehaviors extends AutonomousComponentBehaviors {

        public double getTargetDistance() {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
                return detections.get(0).ftcPose.range;
            }
            return -1; // No target detected
        }

        public int getTargetID() {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (!detections.isEmpty()) {
                return detections.get(0).id;
            }
            return -1; // No target detected
        }

        public boolean isTargetDetected() {
            return !aprilTagProcessor.getDetections().isEmpty();
        }
    }

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final AutonomousLimelightBehaviors auton = new AutonomousLimelightBehaviors();

    public interface LimelightControlStrategy extends IControlStrategy {
        void processVision(Gamepad gamepad);
    }

    private final LimelightControlStrategy strategy;

    public LimelightVision(HardwareMap hardwareMap, LimelightControlStrategy strategy) {
        super(strategy);
        this.strategy = strategy;

        // Initialize AprilTag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Initialize Vision Portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }

    @Override
    public AutonomousLimelightBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.processVision(gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        telemetry.addData("Targets Detected", detections.size());

        if (!detections.isEmpty()) {
            AprilTagDetection target = detections.get(0);

            telemetry.addData("Target ID", target.id);

            if (target.ftcPose != null) {
                telemetry.addData("Distance (inches)", String.format("%.2f", target.ftcPose.range));
                telemetry.addData("Bearing (degrees)", String.format("%.2f", target.ftcPose.bearing));
                telemetry.addData("X (inches)", String.format("%.2f", target.ftcPose.x));
                telemetry.addData("Y (inches)", String.format("%.2f", target.ftcPose.y));
                telemetry.addData("Z (inches)", String.format("%.2f", target.ftcPose.z));
            } else {
                telemetry.addLine("No pose data available");
            }
        } else {
            telemetry.addLine("No targets detected");
        }
    }
}

