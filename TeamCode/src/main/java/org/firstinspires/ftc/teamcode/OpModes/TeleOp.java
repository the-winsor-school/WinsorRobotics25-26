package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group= "Example")
public class TeleOp extends LinearOpMode
{
    Robot robot;
    /**
     * Imagine a world where this is all we need to have in this class~
     * @throws InterruptedException
     */
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        robot = new Wildbots2025(hardwareMap, telemetry);
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.update(gamepad1, gamepad2);
            robot.updateTelemetry();
        }
    }
}
