package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Unstuckinator extends MechComponent{
    public  interface UnstuckinatorControlStrategy extends IControlStrategy {
        public void UCS (CRServo servo, Gamepad gamepad);
    }
    public Unstuckinator(
            HardwareMap hardwareMap,
            String servoName,
            UnstuckinatorControlStrategy strategy) {
        super(strategy);
        spinny = hardwareMap.get(CRServo.class, servoName);
        UnstuckCS = strategy;
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    private final CRServo spinny;
    private final UnstuckinatorControlStrategy UnstuckCS;
    @Override
    public void move(Gamepad gamepad) {
        UnstuckCS.UCS(spinny, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
