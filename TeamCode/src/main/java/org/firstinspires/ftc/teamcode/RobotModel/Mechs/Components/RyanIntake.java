package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RyanIntake extends MechComponent
{
    public RyanIntake(
            HardwareMap hardwareMap,
            String motorName,
            RIControlStrategy ryanintake) {
        super(ryanintake);
        ristrat = ryanintake;
        this.ryanintake = hardwareMap.get(DcMotor.class, motorName);;
    }

    public class AutonomousRyanIntake extends AutonomousComponentBehaviors {
        public void rigogo(){
            ryanintake.setPower(0.67);
        }
        public void rigostop(){
            ryanintake.setPower(0);
        }
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    public interface RIControlStrategy extends IControlStrategy
    {
        void rigo(DcMotor motor, Gamepad gamepad);
    }
    private final DcMotor ryanintake;

    protected RIControlStrategy ristrat;
    @Override
    public void move(Gamepad gamepad) {ristrat.rigo(ryanintake, gamepad);}

    @Override
    public void update(Telemetry telemetry) {

    }
}
