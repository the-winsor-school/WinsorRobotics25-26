package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaFlywheel extends MechComponent {
    public SodaFlywheel(
            HardwareMap hardwareMap,
            String motorName,
            SoFlyControlStrategy soflygo) {
        super(soflygo);
        sodaflywheel = hardwareMap.get(DCMotor.class, motorName);
        this.soflygo = soflygo;
    }

    public class AutonomousSoFly extends AutonomousComponentBehaviors {
        public void StartSoFly(){sodaflywheel.setPower(1);}
        public void StopSoFly(){sodaflywheel.setPower(0);}
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors(){return null;}

    public interface SoFlyControlStrategy extends IControlStrategy {
        void gofly(DcMotor motor, Gamepad gamepad);
    }

    private final DcMotor sodaflywheel;

    protected SoFlyControlStrategy soflygo;

    @Override
    public void move(Gamepad gamepad){soflygo.gofly(sodaflywheel, gamepad);}
    @Override
    public void update(Telemetry telemetry) {

    }
}
