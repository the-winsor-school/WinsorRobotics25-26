package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.MechAssembly;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.RyanMA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class RyanRobot extends Robot {
    public class AutonomousRyanRobot extends AutonomousRobot {
        public final MecanumDrive.AutonomousMecanumDrive driveTrain;

        public final RyanMA.AutonomousRyanMA mechAssembly;

        public AutonomousRyanRobot(
                MecanumDrive.AutonomousMecanumDrive driveTrain,
                RyanMA.AutonomousRyanMA mechAssembly) {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
        }
    }


    private final AprilTagProcessor aprilTagProcessor;
    private final ColorBlobLocatorProcessor purpleBallProcessor;
    private final ColorBlobLocatorProcessor greenBallProcessor;

    private final AutonomousRyanRobot auton;

    @Override
    public AutonomousRyanRobot getAutonomousRobot(){
        return auton;
    }

    public RyanRobot(HardwareMap hardwareMap){
        driveTrain = new MecanumDrive(hardwareMap,
                new MecanumDrive.OrientationConfiguration(
                        DcMotorSimple.Direction.REVERSE,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.REVERSE
                ));
        mechAssembly = new RyanMA(hardwareMap);

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

        auton = new AutonomousRyanRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors()
        );
    }
}
