package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@TeleOp(name="BillyBLUE")
public class BillyTeleOpBLUE extends LinearOpMode {

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BillyRobot(hardwareMap, telemetry, 20);

        telemetry.addLine("Billy Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            robot.update(gamepad1, gamepad2);
            robot.updateTelemetry(telemetry);
            telemetry.update();
        }
    }

}