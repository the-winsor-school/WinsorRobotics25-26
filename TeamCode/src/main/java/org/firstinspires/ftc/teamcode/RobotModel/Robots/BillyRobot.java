package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.BillyMA;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Collections;
import java.util.List;

public class BillyRobot extends Robot {
    public class AutonomousMecanumRobot extends AutonomousRobot
    {
        public final MecanumDrive.AutonomousMecanumDrive driveTrain;
        public final BillyMA.AutonomousBillyMA mechAssembly;

        public AutonomousMecanumRobot(MecanumDrive.AutonomousMecanumDrive driveTrain,
                                      BillyMA.AutonomousBillyMA mechAssembly)
        {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
        }
    }

    private final AprilTagProcessor aprilTag;
    private final AutonomousMecanumRobot auton;
    public final Limelight3A limelight;

    @Override
    public BillyRobot.AutonomousMecanumRobot getAutonomousRobot() {
        return auton;
    }

    public BillyRobot(HardwareMap hardwareMap) {
        driveTrain = new MecanumDrive(hardwareMap, new MecanumDrive.OrientationConfiguration(
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        ));

        mechAssembly = new BillyMA(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        auton = new BillyRobot.AutonomousMecanumRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors());
    }
}