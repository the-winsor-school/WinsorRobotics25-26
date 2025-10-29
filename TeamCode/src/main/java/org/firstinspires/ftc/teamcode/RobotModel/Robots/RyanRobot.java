package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.RyanMA;

public class RyanRobot extends Robot {
    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {

        return null;
    }
    public RyanRobot(HardwareMap hardwareMap){
        // pretty sure Ryan uses a MecanumDrive
        driveTrain = new StandardTankDrive(hardwareMap);
        mechAssembly = new RyanMA(hardwareMap);
    }
}
