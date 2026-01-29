package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;

public class SodaSpindexServo extends MechComponent {
    public interface SoSpinSerControlStrategy extends IControlStrategy {
        public void SCS(SodaSpindexServo servo, Gamepad gamepad);
    }

    public SodaSpindexServo(HardwareMap hardwareMap, String servoName, SoSpinSerControlStrategy strategy) {
        super(strategy);
        spindy = hardwareMap.get(CRServo.class, servoName);
        SpindyCS = strategy;
    }

    public class AutonomousSpindy extends AutonomousComponentBehaviors {
        public void SpindyPos(){spindy.setPower(1);}
        public void SpindyStop(){spindy.setPower(0);}
        public void SpindyNeg(){spindy.setPower(-1);}
    }

    @Override
    public AutonomousSpindy getAutonomousBehaviors() {return auton;}

    private final CRServo spindy;
    private final AutonomousSpindy auton = new AutonomousSpindy();
    private final SoSpinSerControlStrategy SpindyCS;

    @Override
    public void move(Gamepad gamepad) {
        SpindyCS.SCS(this, gamepad);
    }

    public void update(Telemetry telemetry) {

    }
}
