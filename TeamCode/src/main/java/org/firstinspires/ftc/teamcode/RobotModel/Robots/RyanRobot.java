package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.RyanMA;

public class RyanRobot extends Robot {
    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {

        return null;
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
    }
}
