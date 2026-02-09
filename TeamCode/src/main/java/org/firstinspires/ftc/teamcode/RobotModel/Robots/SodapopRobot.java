package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class SodapopRobot extends Robot {

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

    public final AprilTagProcessor aprilTagProcessor;
    public final LimelightVision limelightVision;
    private final VisionPortal visionPortal;

    public SodapopRobot(HardwareMap hardwareMap) {
        driveTrain = new MecanumDrive(
                hardwareMap,
                new MecanumDrive.OrientationConfiguration(
                        DcMotorSimple.Direction.REVERSE,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.REVERSE
                )
        );

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        limelightVision = new LimelightVision(hardwareMap, "limelight",
                (gamepad) -> {
                    // Add Limelight control strategy here
                });

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        mechAssembly = new SodapopMA(hardwareMap);

        auton = new AutonomousSodapopRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors()
        );
    }
}
