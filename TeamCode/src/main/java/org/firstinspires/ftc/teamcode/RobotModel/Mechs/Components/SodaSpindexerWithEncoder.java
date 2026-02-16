/* RobotModel/Mechs/Components/SodaSpindexerWithEncoder.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;

public class SodaSpindexerWithEncoder extends MechComponent {

    public interface SodaSpindexerControlStrategy extends IControlStrategy {
        void controlSpindexer(CRServo servo, DcMotorEx encoder, Gamepad gamepad);
    }

    public SodaSpindexerWithEncoder(HardwareMap hardwareMap,
                                    String servoName,
                                    String encoderMotorName,
                                    SodaSpindexerControlStrategy strategy) {
        super(strategy);
        this.spindexerServo = hardwareMap.get(CRServo.class, servoName);
        this.encoderMotor = hardwareMap.get(DcMotorEx.class, encoderMotorName);
        this.encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.strategy = strategy;
    }

    public class AutonomousSpindexer extends AutonomousComponentBehaviors {

        // CALIBRATION: Run servo at full power and measure time for ONE complete rotation
        // Example: if it takes 2000ms to rotate 360°, set this to 2000
        private static final long FULL_ROTATION_MS = 2000;  // ADJUST THIS VALUE!

        /**
         * Rotate to next position (120 degrees = 1/3 rotation)
         * This advances the spindexer by one section
         */
        public void advanceOnePosition() {
            long timeNeeded = (long) (FULL_ROTATION_MS * (120.0 / 360.0));
            spindexerServo.setPower(1.0);
            ThreadExtensions.TrySleep(timeNeeded);
            spindexerServo.setPower(0);
        }

        /**
         * Rotate backwards by one position (120 degrees)
         */
        public void reverseOnePosition() {
            long timeNeeded = (long) (FULL_ROTATION_MS * (120.0 / 360.0));
            spindexerServo.setPower(-1.0);
            ThreadExtensions.TrySleep(timeNeeded);
            spindexerServo.setPower(0);
        }

        /**
         * Rotate to specific degrees (0-360)
         */
        public void rotateToDegrees(double degrees) {
            long timeNeeded = (long) (FULL_ROTATION_MS * (degrees / 360.0));
            spindexerServo.setPower(1.0);
            ThreadExtensions.TrySleep(timeNeeded);
            spindexerServo.setPower(0);
        }

        public void stop() {
            spindexerServo.setPower(0);
        }
    }

    private final CRServo spindexerServo;
    private final DcMotorEx encoderMotor;
    private final SodaSpindexerControlStrategy strategy;
    private final AutonomousSpindexer auton = new AutonomousSpindexer();

    @Override
    public AutonomousSpindexer getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlSpindexer(spindexerServo, encoderMotor, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Spindexer Servo Power", String.format("%.2f", spindexerServo.getPower()));
        telemetry.addData("Encoder Position", encoderMotor.getCurrentPosition());
        telemetry.addData("Encoder Velocity", String.format("%.0f ticks/sec", encoderMotor.getVelocity()));
    }
}
