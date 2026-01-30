package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaLift extends MechComponent {
    public SodaLift(
            HardwareMap hardwareMap,
            String motorName,
            SodaLiftControlStrategy liftup) {
        super(liftup);
        sodalift = hardwareMap.get(DcMotor.class, motorName);
        this.liftup = liftup;
    }

    public class AutonomousSodaLift extends AutonomousComponentBehaviors {
        public void GoPos() {sodalift.setPower(1);}
        public void GoStop() {sodalift.setPower(0);}
        public void GoNeg() {sodalift.setPower(-1);}
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors(){return null;}

    public interface  SodaLiftControlStrategy extends IControlStrategy {
        void golift(DcMotor motor, Gamepad gamepad);
    }

    private final DcMotor sodalift;

    protected SodaLiftControlStrategy liftup;

    @Override
    public void move(Gamepad gamepad) {liftup.golift(sodalift, gamepad);}

    @Override
    public void update(Telemetry telemetry) {

    }
}
