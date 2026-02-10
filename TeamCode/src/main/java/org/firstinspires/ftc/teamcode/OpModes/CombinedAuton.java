package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.Combined;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "combined auton")
public class CombinedAuton extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy strategy;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BillyRobot(hardwareMap);
        strategy = Combined.track(robot, telemetry, this);

        telemetry.addLine("auton ready");
        telemetry.update();

        waitForStart();
        strategy.execute();
    }
}
