package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewIntakeServo extends MechComponent {
    public class AutonomousNewIntakeBehaviors extends AutonomousComponentBehaviors {
        public void newIntakeUp(){
            newintake.setPower(0.25);
        }
        public void newIntakeDown(){
            newintake.setPower(-0.25);
        }
        public void newIntakeStop(){
            newintake.setPower(0);
        }
    }

    public interface NewIntakeControlStrategy extends IControlStrategy {
        public void NICS (CRServo servo, Gamepad gamepad);
    }

    public NewIntakeServo(
            HardwareMap hardwareMap,
            String servoName,
            NewIntakeControlStrategy strategy) {
        super(strategy);
        newintake = hardwareMap.get(CRServo.class, servoName);
        NewIntakeCS = strategy;
    }

    @Override
    public AutonomousNewIntakeBehaviors getAutonomousBehaviors(){return auton;}
    private final CRServo newintake;
    private final AutonomousNewIntakeBehaviors auton = new AutonomousNewIntakeBehaviors();;
    private final NewIntakeControlStrategy NewIntakeCS;
    @Override
    public void move(Gamepad gamepad) {NewIntakeCS.NICS(newintake, gamepad);}

    @Override
    public void update(Telemetry telemetry) {

    }
}
