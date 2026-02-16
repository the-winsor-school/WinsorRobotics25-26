/* OpModes/SodaTeleOp.java */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@TeleOp(name = "Soda TeleOp", group = "Competition")
public class SodaTeleOp extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SodapopRobot(hardwareMap);

        telemetry.addLine("Soda TeleOp Initialized");
        telemetry.addLine("Drive: Mecanum (left stick)");
        telemetry.addLine("Intake: Triggers");
        telemetry.addLine("Spindexer: X (1 ball), B (3 balls), Arrows (manual)");
        telemetry.addLine("Shooter: Right Bumper (shoot), Left Bumper (unjam)");
        telemetry.addLine("Turret: Right Stick X");
        telemetry.addLine("Hood: Right Stick Y");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.update(gamepad1, gamepad2);
            robot.updateTelemetry(telemetry);
            telemetry.update();
            idle();
        }
    }
}
