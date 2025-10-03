package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends MechComponent
{
    public Shooter(
            HardwareMap hardwareMap,
            String motorName,
            ShooterControlStrategy pewpew) {
        super(pewpew);
        shooter = hardwareMap.get(DcMotor.class, motorName);
        this.pewpew = pewpew;
    }

    public class AutonomousShooterBehavior extends AutonomousComponentBehaviors {
        public void StartShoot(){
            shooter.setPower(1);
        }
        public void StopShoot(){
            shooter.setPower(0);
        }
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    public interface ShooterControlStrategy extends IControlStrategy
    {
        void shoot(DcMotor motor, Gamepad gamepad);
    }
    private final DcMotor shooter;

    protected ShooterControlStrategy pewpew;
    @Override
    public void move(Gamepad gamepad) {
        pewpew.shoot(shooter, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
