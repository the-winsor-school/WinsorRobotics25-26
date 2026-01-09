package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.closeAutonStrat;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "close zone auton")
public class CloseAuton extends LinearOpMode {
    private BillyRobot robot;
    private IAutonStrategy autonStrategy;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BillyRobot(hardwareMap);
        autonStrategy = closeAutonStrat.close(robot.getAutonomousRobot(), telemetry, this);

        telemetry.addLine("Close Auton Ready");
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
