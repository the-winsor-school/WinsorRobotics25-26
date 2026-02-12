package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaTurretAdjuster extends MechComponent{
    public interface SoTuAdControlStrategy extends IControlStrategy {
        public void SACS(Servo servo, Gamepad gamepad);
    }

    public SodaTurretAdjuster(HardwareMap hardwareMap, String servoName, SoTuAdControlStrategy strategy) {
        super(strategy);
        adj = hardwareMap.get(Servo.class, servoName);
        AdjCS = strategy;
    }

    public class AutonomousAdj extends AutonomousComponentBehaviors {
        // In AUTON, this is part of the autoaim portion
        // Also, we need to see what position works
        // AND there are limits...

        public void AdjPos(){adj.setPosition(0);} //EDIT THESE
        public void AdjNeg(){adj.setPosition(1);} //EDIT THESE
    }

    @Override
    public AutonomousAdj getAutonomousBehaviors() {return auton;}

    private final Servo adj;
    private final AutonomousAdj auton = new AutonomousAdj();
    private final SoTuAdControlStrategy AdjCS;

    @Override
    public void move(Gamepad gamepad) {AdjCS.SACS(this, gamepad);}

    public void update(Telemetry telemetry) {

    }
}
