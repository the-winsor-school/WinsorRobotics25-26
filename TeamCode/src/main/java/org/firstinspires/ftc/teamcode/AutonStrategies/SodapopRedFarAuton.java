package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class SodapopRedFarAuton {
    public static IAutonStrategy ShootAtRed(SodapopRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("=== SODA AUTONOMOUS STARTED ===");
            telemetry.addData("Target", "Red AprilTag ID 24 For Far");
            telemetry.addData("Strategy", "Find Motif->Find Goal Tag-> Shoot Round->Drive to 2nd Artifact Line->Intake Artifacts->Go to Launch Zone->Shoot->Go to Gate->Intake From Gate->Go to Launch Zone->Shoot->Repeat->Shoot to Motif->Go to Corner Artifacts-> Intake Corner Artifacts->Go to Launch Zone-> Shoot to Motif->Go to 3rd Line Artifacts->Intake Artifacts from 3rd Line->Go to Close Launch Zone-> Move + Shoot Round to Motif->Stop");
            telemetry.update();

            //IState currentState = goForwardFor(4000, robot, telemetry);

            while(opMode.opModeIsActive() && currentState != null)
            {
                currentState = currentState.execute();
                opMode.idle();
            }

            telemetry.clear();
            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
            telemetry.update();
        };
    }
}
