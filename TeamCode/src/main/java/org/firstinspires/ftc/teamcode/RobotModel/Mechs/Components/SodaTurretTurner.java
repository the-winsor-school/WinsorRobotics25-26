/* RobotModel/Mechs/Components/SodaTurretTurner.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaTurretTurner extends MechComponent {

    public interface SodaTurretTurnerControlStrategy extends IControlStrategy {
        void controlTurret(CRServo servo, Gamepad gamepad);
    }

    public SodaTurretTurner(HardwareMap hardwareMap,
                            String servoName,
                            SodaTurretTurnerControlStrategy strategy) {
        super(strategy);
        this.turretServo = hardwareMap.get(CRServo.class, servoName);
        this.strategy = strategy;
    }

    public class AutonomousTurretTurner extends AutonomousComponentBehaviors {

        /**
         * Rotate turret clockwise (positive power)
         */
        public void rotateClockwise(double power) {
            turretServo.setPower(Math.max(-1, Math.min(1, power)));
        }

        /**
         * Rotate turret counter-clockwise (negative power)
         */
        public void rotateCounterClockwise(double power) {
            turretServo.setPower(-Math.max(-1, Math.min(1, power)));
        }

        /**
         * Rotate turret at full speed clockwise
         */
        public void spinFastClockwise() {
            turretServo.setPower(1.0);
        }

        /**
         * Rotate turret at full speed counter-clockwise
         */
        public void spinFastCounterClockwise() {
            turretServo.setPower(-1.0);
        }

        /**
         * Stop turret rotation
         */
        public void stop() {
            turretServo.setPower(0);
        }
    }

    private final CRServo turretServo;
    private final SodaTurretTurnerControlStrategy strategy;
    private final AutonomousTurretTurner auton = new AutonomousTurretTurner();

    @Override
    public AutonomousTurretTurner getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlTurret(turretServo, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Turret Servo Power", String.format("%.2f", turretServo.getPower()));
    }
}
