package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class BillyRobot extends Robot {
    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {

        return null;
    }


    public BillyRobot(HardwareMap hardwareMap) {
        driveTrain = new MecanumDrive(hardwareMap, new MecanumDrive.OrientationConfiguration(
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE
        ));

        mechAssembly = new BillyMA(hardwareMap);
    }
}