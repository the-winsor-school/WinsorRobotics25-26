package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class SodapopRobot extends Robot{

    public class AutonomousSodapopRobot extends AutonomousRobot {
        public final MecanumDrive.AutonomousMecanumDrive driveTrain;
        public final SodapopMA.AutonomousSodapopMA mechAssembly;

        public AutonomousSodapopRobot(
                MecanumDrive.AutonomousMecanumDrive driveTrain,
                SodapopMA.AutonomousSodapopMA mechAssembly) {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
        }
    }

    private final AutonomousSodapopRobot auton;

    @Override
    public AutonomousSodapopRobot getAutonomousRobot() {
        return auton;
    }

    private final AprilTagProcessor aprilTagProcessor;
    private final ColorBlobLocatorProcessor purpleBallProcessor;
    private final ColorBlobLocatorProcessor greenBallProcessor;

    public SodapopRobot(HardwareMap hardwareMap) {
        // Initialize drive train with proper motor orientation
        driveTrain = new MecanumDrive(hardwareMap,
                new MecanumDrive.OrientationConfiguration(
                        DcMotorSimple.Direction.REVERSE,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.REVERSE
                ));

        // Initialize backup AprilTag system (webcam-based)
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

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

        // Set up backup webcam vision system
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(purpleBallProcessor)
                .addProcessor(greenBallProcessor)
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(org.firstinspires.ftc.vision.VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        // Initialize the SodapopMA (which includes LimelightForSoda)
        mechAssembly = new SodapopMA(hardwareMap);

        // Create autonomous robot
        auton = new AutonomousSodapopRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors()
        );
    }

    // Convenience method to access Limelight through the mech assembly
    public LimelightForSoda getLimelightForSoda() {
        return ((SodapopMA) mechAssembly).limelightForSoda;
    }

    // Convenience method to access color processors for backup vision
    public ColorBlobLocatorProcessor getPurpleBallProcessor() {
        return purpleBallProcessor;
    }

    public ColorBlobLocatorProcessor getGreenBallProcessor() {
        return greenBallProcessor;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
}
