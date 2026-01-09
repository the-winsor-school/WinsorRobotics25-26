package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.leftSideAutonStrat;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "close zone auton")
public class LeftSideAuton extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy autonStrategy;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BillyRobot(hardwareMap);
        autonStrategy = leftSideAutonStrat.left(robot.getAutonomousRobot(), telemetry, this);

        telemetry.addLine("Close Auton Ready");
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
