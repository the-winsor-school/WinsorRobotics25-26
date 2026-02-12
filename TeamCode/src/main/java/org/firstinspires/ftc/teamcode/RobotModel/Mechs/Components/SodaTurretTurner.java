package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaTurretTurner extends MechComponent {
    public interface SoTuTuControlStrategy extends IControlStrategy {
        public void STCS(CRServo servo, Gamepad gamepad);
    }

    public SodaTurretTurner(HardwareMap hardwareMap, String servoName, SoTuTuControlStrategy strategy) {
        super(strategy);
        turndy = hardwareMap.get(CRServo.class, servoName);
        TurndyCS = strategy;
    }

    public class AutonomousTurndy extends AutonomousComponentBehaviors {
        // I don't think calibration is necessary, BUT...
        // In AUTON, this is part of the autoaim portion
        // TELEOP is very simple for this component

        public void TurndyPos(){turndy.setPower(1);}
        public void TurndyStop(){turndy.setPower(0);}
        public void TurndyNeg(){turndy.setPower(-1);}
    }

    @Override
    public AutonomousTurndy getAutonomousBehaviors() {return auton;}

    private final CRServo turndy;
    private final AutonomousTurndy auton = new AutonomousTurndy();
    private final SoTuTuControlStrategy TurndyCS;

    @Override
    public void move(Gamepad gamepad) {TurndyCS.STCS(this, gamepad);}

    public void update(Telemetry telemetry) {

    }
}