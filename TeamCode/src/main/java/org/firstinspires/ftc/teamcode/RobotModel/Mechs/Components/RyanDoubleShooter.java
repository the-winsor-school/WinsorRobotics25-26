package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.MechComponent.AutonomousComponentBehaviors;

public class RyanDoubleShooter extends MechComponent{
    public RyanDoubleShooter(
            HardwareMap hardwareMap,
            String motorNameLeft,
            String motorNameRight,
            DoubleShooterControlStrategy kapow) {
        super(kapow);
        doubleshooterleft = hardwareMap.get(DcMotor.class, motorNameLeft);
        doubleshooterright = hardwareMap.get(DcMotor.class, motorNameRight);
        this.kapow = kapow;
    }

    public class AutonomousDoubleShooterBehavior extends AutonomousComponentBehaviors {
        public void DoubleShootersGo(){
            doubleshooterleft.setPower(0.67);
            doubleshooterright.setPower(0.67);
        }
        public void DoubleShootersStop(){
            doubleshooterleft.setPower(0);
            doubleshooterright.setPower(0);
        }
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    @Override
    void move(Gamepad gamepad) {
        
    }

    @Override
    void update(Telemetry telemetry) {

    }

    public interface DoubleShooterControlStrategy extends MechComponent.IControlStrategy {
        void doubleshoot(DcMotor motorleft, DcMotor motorright, Gamepad gamepad);
    }

    private final DcMotor doubleshooterleft;
    private final DcMotor doubleshooterright;

    protected DoubleShooterControlStrategy kapow;

    @Override
    public void
    )
}