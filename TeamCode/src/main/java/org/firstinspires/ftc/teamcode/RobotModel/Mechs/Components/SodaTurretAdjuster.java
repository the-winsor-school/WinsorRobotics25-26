/* RobotModel/Mechs/Components/SodaTurretAdjuster.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaTurretAdjuster extends MechComponent {

    public interface SodaTurretAdjusterControlStrategy extends IControlStrategy {
        void adjustHood(Servo servo, Gamepad gamepad);
    }

    public SodaTurretAdjuster(HardwareMap hardwareMap,
                              String servoName,
                              SodaTurretAdjusterControlStrategy strategy) {
        super(strategy);
        this.hoodServo = hardwareMap.get(Servo.class, servoName);
        this.strategy = strategy;
    }

    public class AutonomousHoodAdjuster extends AutonomousComponentBehaviors {

        /**
         * Set hood to specific angle (0-90 degrees)
         * @param angle 0 to 90
         */
        public void setHoodAngle(double angle) {
            // Convert angle (0-90) to servo position (0-1)
            double position = angle / 90.0;
            hoodServo.setPosition(Math.max(0, Math.min(1, position)));
        }

        /**
         * Hood flat (0 degrees)
         */
        public void hoodFlat() {
            hoodServo.setPosition(0.0);
        }

        /**
         * Hood angled up (90 degrees)
         */
        public void hoodMax() {
            hoodServo.setPosition(1.0);
        }

        /**
         * Hood at 45 degrees (middle position)
         */
        public void hoodMid() {
            hoodServo.setPosition(0.5);
        }
    }

    private final Servo hoodServo;
    private final SodaTurretAdjusterControlStrategy strategy;
    private final AutonomousHoodAdjuster auton = new AutonomousHoodAdjuster();

    @Override
    public AutonomousHoodAdjuster getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.adjustHood(hoodServo, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        double position = hoodServo.getPosition();
        double angle = position * 90.0;
        telemetry.addData("Hood Servo Position", String.format("%.2f", position));
        telemetry.addData("Hood Angle (est.)", String.format("%.1f°", angle));
    }
}
