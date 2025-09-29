package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;

public class FranklinRobot extends Robot {
    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {

        return null;
    }
    public FranklinRobot(HardwareMap hardwareMap){
        driveTrain = new StandardTankDrive(hardwareMap);
        mechAssembly = new FranklinMA(hardwareMap);
    }
}
