package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions.LimelightPosePoller;
import org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions.LimelightResultsClient;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.LimelightTestRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@TeleOp(name = "Limelight Test", group = "Testing")
public class LimelightTestOpMode extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        LimelightResultsClient client = new LimelightResultsClient("192.168.43.11");
        LimelightPosePoller poller = new LimelightPosePoller(this, client, 250);
        Robot robot = new LimelightTestRobot(poller);

        waitForStart();
        poller.start();

        try
        {
            while (opModeIsActive())
            {
                // No drivetrain/mech commands; just push Limelight telemetry.
                robot.updateTelemetry(telemetry);
                idle();
            }
        }
        finally
        {
            poller.stop();
        }
    }
}
