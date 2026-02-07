package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class closeAutonStrat {
    public static IAutonStrategy close(BillyRobot robot,
                                       Telemetry telemetry,
                                       LinearOpMode opMode)
    {
        return() ->
        {
            IState currentState = lookForTag(robot, telemetry);
        };
    }

    public static IState lookForTag(BillyRobot robot, Telemetry telemetry)
    {
        Limelight3A limelight = robot.limelight;

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        }
        else
        {
            telemetry.addData("Limelight", "No Targets");
        }

        return turnToTag(robot, telemetry);
    }

    public static IState turnToTag(BillyRobot robot, Telemetry telemetry)
    {
        return lookForTag(robot, telemetry);
    }
}
