package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.rightSideAutonStrat;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "right side auton")
public class RightSideAuton extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy autonStrategy;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BillyRobot(hardwareMap);
        autonStrategy = rightSideAutonStrat.right(robot.getAutonomousRobot(), telemetry, this);

        telemetry.addLine("Auton Ready");
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
