package org.firstinspires.ftc.teamcode.AutonStrategies;

import static org.firstinspires.ftc.teamcode.AutonStrategies.ATagL1Strategy.lookForTag;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Turret;

public class LimelightAutoTarget {
    private IState currentState = lookForTag();
    public void updateState()
    {
        currentState = currentState.execute();
    }
    private final int targetTagId;
    private final Limelight3A limelight;
    private final Turret.AutonomousTurretBehaviors turret;
    private final Telemetry telemetry;

    public LimelightAutoTarget(
            Limelight3A limelight,
            Turret.AutonomousTurretBehaviors turret,
            Telemetry telemetry,
            int tagId)
    {
        this.limelight = limelight;
        this.turret = turret;
        this.telemetry = telemetry;
        this.targetTagId = tagId;
    }

    public IState rotateCCW(double tx){
        return () ->
        {
            double power = 1;
            if(tx > -10)
                power = -tx / 10.0;
            turret.setPower(power);
            telemetry.addData("Turret CCW: ", power);
            telemetry.update();
            return lookForTag();
        };
    }
    public IState rotateCW(double tx) {
        return () ->
        {
            double power = 1;
            if (tx < 10)
                power = -tx / 10.0;
            turret.setPower(power);
            telemetry.addData("Turret CW: ", power);
            telemetry.update();
            return lookForTag();
        };
    }

    public IState stopTurret() {
        return () ->
        {
            turret.stop();
            telemetry.addLine("Turret Stopped");
            telemetry.update();
            return lookForTag();
        };
    }

    public IState lookForTag() {
        return()->
        {
            LLResultTypes.FiducialResult tag =
                    LimelightExtensions.tryGetFiducial(
                            limelight,
                            targetTagId);

            if(tag == null)
            {
                telemetry.addLine(
                        "Tag" + targetTagId + " not found.");
                return stopTurret();
            }
            double tx = tag.getTargetXDegrees();
            if(tx < -2) return rotateCCW(tx);
            else if(tx > 2) return rotateCW(tx);
            else            return stopTurret();
        };
    }
}


//finish for hw writing the slides- not include the reflection
