package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SodaFlywheel extends MechComponent {

    public class AutonomousSoFly extends AutonomousComponentBehaviors {
        /**
         * Start flywheel at target velocity (ticks per second)
         */
        public void startSoFly() {
            setTargetVelocity(TARGET_VELOCITY);
        }

        public void stopSoFly() {
            setTargetVelocity(0);
        }

        /**
         * Set flywheel to specific velocity
         */
        public void setVelocity(double ticksPerSecond) {
            setTargetVelocity(ticksPerSecond);
        }

        /**
         * Check if flywheel is at target velocity
         */
        public boolean isAtTargetVelocity() {
            double currentVelocity = sodaflywheel.getVelocity();
            return Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE;
        }

        /**
         * Get current velocity
         */
        public double getCurrentVelocity() {
            return sodaflywheel.getVelocity();
        }

        /**
         * Get target velocity
         */
        public double getTargetVelocity() {
            return targetVelocity;
        }
    }

    public interface SoFlyControlStrategy extends IControlStrategy {
        void gofly(DcMotorEx motor, Gamepad gamepad);
    }

    private final DcMotorEx sodaflywheel;
    protected SoFlyControlStrategy soflygo;
    private final AutonomousSoFly auton = new AutonomousSoFly();

    // PID Tuning parameters
    private static final double TARGET_VELOCITY = 2000.0;  // ticks per second
    private static final double VELOCITY_TOLERANCE = 50.0;  // ticks per second

    // PID Coefficients (tune these!)
    private static final double Kp = 0.001;
    private static final double Ki = 0.0001;
    private static final double Kd = 0.00005;

    private double targetVelocity = 0;
    private double integralError = 0;
    private double lastError = 0;

    public SodaFlywheel(
            HardwareMap hardwareMap,
            String motorName,
            SoFlyControlStrategy soflygo) {
        super(soflygo);

        // Get motor as DcMotorEx for velocity control
        sodaflywheel = hardwareMap.get(DcMotorEx.class, motorName);
        this.soflygo = soflygo;

        // Set to RUN_USING_ENCODER for PID
        sodaflywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sodaflywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public AutonomousSoFly getAutonomousBehaviors() {
        return auton;
    }

    /**
     * Set target velocity and apply PID control
     */
    private void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    /**
     * Calculate PID output
     */
    private double calculatePIDOutput() {
        double currentVelocity = sodaflywheel.getVelocity();
        double error = targetVelocity - currentVelocity;

        // Proportional term
        double pTerm = Kp * error;

        // Integral term (with anti-windup)
        integralError += error;
        if (Math.abs(integralError) > 1000) {
            integralError = Math.copySign(1000, integralError);
        }
        double iTerm = Ki * integralError;

        // Derivative term
        double dTerm = Kd * (error - lastError);
        lastError = error;

        // Total output
        double output = pTerm + iTerm + dTerm;

        // Clamp to [-1, 1]
        return Math.max(-1.0, Math.min(1.0, output));
    }

    @Override
    public void move(Gamepad gamepad) {
        soflygo.gofly(sodaflywheel, gamepad);

        // Apply PID control every cycle
        double pidOutput = calculatePIDOutput();
        sodaflywheel.setPower(pidOutput);
    }

    @Override
    public void update(Telemetry telemetry) {
        double currentVelocity = sodaflywheel.getVelocity();
        double error = targetVelocity - currentVelocity;

        telemetry.addLine("=== Soda Flywheel ===");
        telemetry.addData("Target Velocity", String.format("%.0f tps", targetVelocity));
        telemetry.addData("Current Velocity", String.format("%.0f tps", currentVelocity));
        telemetry.addData("Error", String.format("%.0f tps", error));
        telemetry.addData("At Target", auton.isAtTargetVelocity() ? "✓ YES" : "✗ NO");
        telemetry.addData("Motor Power", String.format("%.2f", sodaflywheel.getPower()));
    }
}
