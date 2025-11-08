package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;

@Autonomous(name = "Franklin Auton")
public class FranklinAuton extends LinearOpMode {
    private FranklinRobot robot;
    private IAutonStrategy autonStrategy;
    public FranklinAuton(){
        robot = new FranklinRobot(hardwareMap);
        autonStrategy = FranklinAutonStrategy.RedGoalAutonomous(robot.getAutonomousRobot());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        autonStrategy.execute();
    }
}
