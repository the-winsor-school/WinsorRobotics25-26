package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DoubleShooter extends MechComponent
{
    public DoubleShooter(
            HardwareMap hardwareMap,
            String motorNameF, String motorNameB,
            ShooterControlStrategy pewpew,
            ShooterTelemetryStrategy telemetryStrategy) {
        super(pewpew);
        shooterF = hardwareMap.get(DcMotor.class, motorNameF);
        shooterB = hardwareMap.get(DcMotor.class, motorNameB);
        this.pewpew = pewpew;
        this.telemetryStrategy = telemetryStrategy;
    }

    public class AutonomousShooterBehavior extends AutonomousComponentBehaviors {
        public void StartShoot() {
            shooterF.setPower(0.76);
            shooterB.setPower(-0.76);
        }
        public void StopShoot(){
            shooterF.setPower(0);
            shooterB.setPower(0);
        }

        public void setPower(double power){
            shooterF.setPower(power);
            shooterB.setPower(-power);
        }
    }

    @Override
    public <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors() {
        // Return a new instance of the autonomous behavior so callers get a working object,
        // not null. Cast is safe because AutonomousShooterBehavior extends AutonomousComponentBehaviors.
        return (T) new AutonomousShooterBehavior();
    }

    public interface ShooterControlStrategy extends IControlStrategy
    {
        void shoot(DcMotor motorF, DcMotor motorB, Gamepad gamepad);
    }

    public interface ShooterTelemetryStrategy
    {
        public void update(DcMotor motorF, DcMotor motorB, Telemetry telemetry);
    }
    private final DcMotor shooterF;
    private final DcMotor shooterB;

    protected ShooterControlStrategy pewpew;
    protected ShooterTelemetryStrategy telemetryStrategy;

    @Override
    public void move(Gamepad gamepad) {
        pewpew.shoot(shooterF, shooterB, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
