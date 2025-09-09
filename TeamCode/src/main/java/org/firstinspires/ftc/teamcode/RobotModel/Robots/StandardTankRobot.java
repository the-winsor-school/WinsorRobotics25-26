package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;

public class StandardTankRobot extends Robot
{
    public StandardTankRobot(HardwareMap hardwareMap)
    {
        driveTrain = new StandardTankDrive(hardwareMap);
    }

    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {
        return null;
    }
}
