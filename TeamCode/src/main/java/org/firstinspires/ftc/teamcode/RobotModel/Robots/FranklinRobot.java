package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class FranklinRobot extends Robot {
    public class AutonomousFranklinRobot extends AutonomousRobot {
        public final StandardTankDrive.AutonomousTankDrive driveTrain;
        public final FranklinMA.AutonomousFranklinMA mechAssembly;

        public AutonomousFranklinRobot(
                StandardTankDrive.AutonomousTankDrive driveTrain,
                FranklinMA.AutonomousFranklinMA mechAssembly) {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
        }
    }

    private final AutonomousFranklinRobot auton;

    @Override
    public AutonomousFranklinRobot getAutonomousRobot() {
        return auton; // Return the stored instance instead of null
    }

    public final AprilTagProcessor aprilTagProcessor;
    private final ColorBlobLocatorProcessor purpleBallProcessor;
    private final ColorBlobLocatorProcessor greenBallProcessor;

    public FranklinRobot(HardwareMap hardwareMap) {
        driveTrain = new StandardTankDrive(hardwareMap);

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

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(purpleBallProcessor)
                .addProcessor(greenBallProcessor)
                .addProcessor(aprilTagProcessor)
                // this is supposed to make it show up on the driverstation....
                // it works, but you can only do it at a very specific time...
                .setStreamFormat(org.firstinspires.ftc.vision.VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
        mechAssembly = new FranklinMA(hardwareMap);

        auton = new AutonomousFranklinRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors()
        );
    }
}
