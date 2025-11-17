package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonStrategies.RyanAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.RyanRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;

@Autonomous(name = "Ryan Auton")
public class RyanAuton extends LinearOpMode {
    private RyanRobot robot;
    private IAutonStrategy autonStrategy;
    public RyanAuton(){
        robot = new RyanRobot(hardwareMap);
        autonStrategy = RyanAutonStrategy.BasicAutonomous(robot.getAutonomousRobot());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        autonStrategy.execute();
    }
}
