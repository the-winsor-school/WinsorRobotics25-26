package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class BallDetectionComponent extends MechComponent{
    public class AutonomousBallDetection extends AutonomousComponentBehaviors {
        public boolean isPurpleBallDetected() {
            return purpleBallProcessor.getBlobs().size() > 0;
        }
        public boolean isGreenBallDetected() {
            return greenBallProcessor.getBlobs().size() > 0;
        }

        /**
         * returns -1 if no ball is detected
         * @return
         */
        public double getPurpleBallBearing() {
            if (isPurpleBallDetected()) {
                return purpleBallProcessor.getBlobs().get(0).getBoxFit().center.x;
            }
            return -1;
        }

        /**
         * returns -1 if no ball is detected
         * @return
         */
        public double getGreenBallBearing() {
            if (isGreenBallDetected()) {
                return greenBallProcessor.getBlobs().get(0).getBoxFit().center.x;
            }
            return -1;
        }
    }

    public interface BallDetectionStrategy extends IControlStrategy {
        void processDetection(ColorBlobLocatorProcessor purpleProcessor, ColorBlobLocatorProcessor greenProcessor, Gamepad gamepad);
    }
    private final VisionPortal VisionPortal;
    private final ColorBlobLocatorProcessor purpleBallProcessor;
    private final ColorBlobLocatorProcessor greenBallProcessor;
    private final BallDetectionStrategy strategy;
    private final AutonomousBallDetection auton = new AutonomousBallDetection();

    public BallDetectionComponent(HardwareMap hardwareMap,
                                  String cameraName,
                                  BallDetectionStrategy strategy) {
        super(strategy);
        this.strategy = strategy;

        purpleBallProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .build();

        greenBallProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .build();

        VisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(purpleBallProcessor)
                .addProcessor(greenBallProcessor)
                .build();
    }

    @Override
    public AutonomousBallDetection getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.processDetection(purpleBallProcessor, greenBallProcessor, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleBallProcessor.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenBallProcessor.getBlobs();
        telemetry.addData("Purple Balls Detected", purpleBlobs.size());
        telemetry.addData("Green Balls Detected", greenBlobs.size());

        if (purpleBlobs.size()>0) {
            ColorBlobLocatorProcessor.Blob largestBlob = purpleBlobs.get(0);
            telemetry.addData("Purple Ball X Position", largestBlob.getBoxFit().center.x);
            telemetry.addData("Purple Ball Y Position", largestBlob.getBoxFit().center.y);
            telemetry.addData("Ball Area", largestBlob.getContourArea());
        }

        if (greenBlobs.size()>0) {
            ColorBlobLocatorProcessor.Blob largestBlob = greenBlobs.get(0);
            telemetry.addData("Green Ball X Position", largestBlob.getBoxFit().center.x);
            telemetry.addData("Green Ball Y Position", largestBlob.getBoxFit().center.y);
            telemetry.addData("Ball Area", largestBlob.getContourArea());
        }
    }
}
