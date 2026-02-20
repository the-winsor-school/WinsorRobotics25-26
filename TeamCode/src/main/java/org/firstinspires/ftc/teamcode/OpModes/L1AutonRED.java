package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.ATagL1Strategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "Level 1 Auton RED")
public class L1AutonRED extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy strategy;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BillyRobot(hardwareMap, telemetry, 24);
        strategy = ATagL1Strategy.track(robot, telemetry, this, 24);

        telemetry.addLine("auton ready");
        telemetry.update();

        waitForStart();
        strategy.execute();
    }
}
