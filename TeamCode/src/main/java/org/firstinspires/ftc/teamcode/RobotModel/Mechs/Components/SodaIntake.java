package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaIntake extends MechComponent {
    public SodaIntake(
            HardwareMap hardwareMap,
            String motorName,
            SoInControlStrategy sodaintake) {
        super(sodaintake);
        soinstrat = sodaintake;
        this.sodaintake = hardwareMap.get(DcMotor.class, motorName);
    }

    public class AutonomousSodaIntake extends AutonomousComponentBehaviors {
        public void soingo(){sodaintake.setPower(1);}
        public void soinstop(){sodaintake.setPower(0);}
        public void soinreverse(){sodaintake.setPower(-1);}
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {return null;}

    public interface SoInControlStrategy extends IControlStrategy {
        void soinisgooo(DcMotor motor, Gamepad gamepad);
    }
    private final DcMotor sodaintake;

    protected SoInControlStrategy soinstrat;

    @Override
    public void move(Gamepad gamepad) {soinstrat.soinisgooo(sodaintake, gamepad);}

    @Override
    public void update(Telemetry telemetry) {

    }
}
