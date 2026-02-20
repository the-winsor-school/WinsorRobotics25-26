package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.ATagL2Strategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "Level 2 Auton RED")
public class L2AutonRED extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy strategy;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BillyRobot(hardwareMap, telemetry, 24);
        strategy = ATagL2Strategy.track(robot, telemetry, this, 24);

        telemetry.addLine("auton ready");
        telemetry.update();

        waitForStart();
        strategy.execute();
    }
}
