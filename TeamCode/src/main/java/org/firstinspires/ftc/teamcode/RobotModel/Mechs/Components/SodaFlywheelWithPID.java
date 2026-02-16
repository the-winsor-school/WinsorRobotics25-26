/* RobotModel/Mechs/Components/SodaFlywheelWithPID.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaFlywheelWithPID extends MechComponent {

    public interface SodaFlywheelControlStrategy extends IControlStrategy {
        void controlFlywheel(DcMotorEx motor, SodaFlywheelWithPID flywheel, Gamepad gamepad);
    }

    public SodaFlywheelWithPID(HardwareMap hardwareMap,
                               String motorName,
                               SodaFlywheelControlStrategy strategy) {
        super(strategy);
        this.flywheelMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.strategy = strategy;
    }

    public class AutonomousFlywheel extends AutonomousComponentBehaviors {

        /**
         * Set flywheel to a specific power (0-1)
         */
        public void setFlywheelPower(double power) {
            flywheelMotor.setPower(Math.max(-1, Math.min(1, power)));
        }

        /**
         * Run flywheel at standard shooting speed (0.8 power = ~4000-5000 RPM)
         */
        public void shootingSpeed() {
            flywheelMotor.setPower(0.8);
        }

        /**
         * Stop flywheel
         */
        public void stop() {
            flywheelMotor.setPower(0);
        }

        /**
         * Get current RPM (approximate, based on encoder velocity)
         * This is a rough estimate: velocity is in ticks/sec
         */
        public double getRPM() {
            double ticksPerSec = flywheelMotor.getVelocity();
            double ticksPerRev = 537.7;  //Need to adjust for REV
            double revsPerSec = ticksPerSec / ticksPerRev;
            return revsPerSec * 60.0;  // Convert to RPM
        }

        /**
         * Simple velocity control: ramp power until target RPM is reached
         * @param targetRPM target RPM (e.g., 4500)
         * @param maxPower maximum power to apply (e.g., 0.85)
         */
        public void spinToRPM(double targetRPM, double maxPower) {
            double currentRPM = getRPM();
            double error = targetRPM - currentRPM;

            // Simple proportional control
            double kP = 0.0002;  // Proportional gain (adjust if needed)
            double power = error * kP;

            // Clamp power between 0 and maxPower
            power = Math.max(0, Math.min(maxPower, power));

            flywheelMotor.setPower(power);
        }
    }

    private final DcMotorEx flywheelMotor;
    private final SodaFlywheelControlStrategy strategy;
    private final AutonomousFlywheel auton = new AutonomousFlywheel();

    @Override
    public AutonomousFlywheel getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlFlywheel(flywheelMotor, this, gamepad);
    }

    /**
     * Public method for getting RPM (used by control strategies)
     */
    public double getCurrentRPM() {
        return auton.getRPM();
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Flywheel Power", String.format("%.2f", flywheelMotor.getPower()));
        telemetry.addData("Flywheel RPM (est.)", String.format("%.0f", getCurrentRPM()));
        telemetry.addData("Flywheel Velocity (ticks/sec)", String.format("%.0f", flywheelMotor.getVelocity()));
    }
}

