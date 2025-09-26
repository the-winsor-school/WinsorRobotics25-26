package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModel.Robots.AppleRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AppleTeleOp", group= "Example")
public class AppleTeleOp extends LinearOpMode
{

    Robot robot;
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        robot = new AppleRobot(hardwareMap);
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.update(gamepad1, gamepad2);
            robot.updateTelemetry(telemetry);

            telemetry.update();
            //double time = this.getRuntime();
        }
    }
}
