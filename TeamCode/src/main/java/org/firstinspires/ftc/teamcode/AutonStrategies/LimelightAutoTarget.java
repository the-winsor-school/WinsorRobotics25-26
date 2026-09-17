package org.firstinspires.ftc.teamcode.AutonStrategies;

import static org.firstinspires.ftc.teamcode.AutonStrategies.ATagL1Strategy.lookForTag;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Turret;

/**
 * State machine that rotates the turret until the target AprilTag is centred in
 * the Limelight's field of view. Previously held a raw {@code Telemetry} reference
 * and called {@code telemetry.update()} between state transitions (Susan Zuo —
 * Bug #2: "mid-cycle {@code telemetry.update()} in state machines", and Bug #6:
 * "autonomous strategies held raw telemetry references, bypassing the object
 * model"). All reporting now goes through
 * {@code turret.reportStatus/reportData} so the single-flush rule is respected.
 */
public class LimelightAutoTarget extends StateMachine {
    /**
     * The targetTagId is the AprilTag ID to keep track of.
     * Limelight is the Limelight3A sensor that senses for the AprilTag.
     * The turret rotates to look for the AprilTag.
     */
    private final int targetTagId;
    private final Limelight3A limelight;
    private final Turret.AutonomousTurretBehaviors turret;

    /**
     * @param limelight  the Limelight3A sensor
     * @param turret     the live autonomous turret behavior — carries its own
     *                   telemetry reference, so no raw {@code Telemetry} arg is
     *                   needed here (Susan Zuo — Bug #6)
     * @param tagId      AprilTag ID to track
     * the currentState is set to the state lookForTag();
     */
    public LimelightAutoTarget(
            Limelight3A limelight,
            Turret.AutonomousTurretBehaviors turret,
            int tagId)
    {
        this.limelight = limelight;
        this.turret = turret;
        this.targetTagId = tagId;
        currentState = lookForTag();
    }

    /**
     * The state sets the power to 1.
     * If tx > -10, the power is set to -tx / 10.0.
     * The turret returns "Turret CCW" and the power
     * @param tx is the current tx in pixels from the crosshair
     * @return the state lookForTag();
     */

    public IState rotateCCW(double tx){
        return () ->
        {
            double power = 1;
            if(tx > -10)
                power = -tx / 10.0;
            turret.setPower(power);
            turret.reportData("Turret CCW", power);
            return lookForTag();
        };
    }

    /**
     * The state assigns power to 1.
     * If the tx is less than 10; the power is -tx / 10.0.
     * It sets the turret to the power.
     * The state returns "Turret CW" and the power.
     * @param tx is the current tx in pixels from the crosshair
     * @return the state lookForTag();
     */
    public IState rotateCW(double tx) {
        return () ->
        {
            double power = 1;
            if (tx < 10)
                power = -tx / 10.0;
            turret.setPower(power);
            turret.reportData("Turret CW", power);
            return lookForTag();
        };
    }

    /**
     * Turret set to 0 power, prints on telemetry "Turret Stopped"
     * @return the nextState, lookForTag();
     */

    public IState stopTurret() {
        return () ->
        {
            turret.stop();
            turret.reportStatus("Turret Stopped");
            return lookForTag();
        };
    }

    /**
     * The IState returns the AprilTagID, and if it is not there, it returns null.
     * If the tag is null, it prints on telemetry the aprilTagID and that it is not found.
     * The variable tx is set to the current tx in degrees away from the crosshair
     *
     * @return If the tag is null, the IState goes to the next state StopTurret.
     * The state rotateCCW(tx) is returned if the current tx is less than -2.
     * The state rotateCW(tx) is returned if the current tx is greater than -2.
     * The state stopTurret() is returned if tx = -2.
     */
    public IState lookForTag() {
        return()->
        {
            LLResultTypes.FiducialResult tag =
                    LimelightExtensions.tryGetFiducial(
                            limelight,
                            targetTagId);

            if(tag == null)
            {
                turret.reportStatus("Tag " + targetTagId + " not found.");
                return stopTurret();
            }
            double tx = tag.getTargetXDegrees();
            if(tx < -2) return rotateCCW(tx);
            else if(tx > 2) return rotateCW(tx);
            else            return stopTurret();
        };
    }
}
