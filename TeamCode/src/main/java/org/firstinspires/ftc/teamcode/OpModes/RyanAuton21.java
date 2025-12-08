package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.RyanAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.RyanRobot;

@Autonomous(name = "Ryan Auton")
public class RyanAuton21 extends LinearOpMode {
    private RyanRobot robot;
    private IAutonStrategy autonStrategy;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RyanRobot(hardwareMap);
        autonStrategy = RyanAutonStrategy.Green(
                robot,
                telemetry,
                this
        );

        telemetry.addLine("Ryan Auton Ready");
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
