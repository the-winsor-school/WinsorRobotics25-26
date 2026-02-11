package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;

public class SodaSpindexServo extends MechComponent {
    public interface SoSpinSerControlStrategy extends IControlStrategy {
        public void SCS(SodaSpindexServo servo, Gamepad gamepad);
    }

    public SodaSpindexServo(HardwareMap hardwareMap, String servoName, SoSpinSerControlStrategy strategy) {
        super(strategy);
        spindy = hardwareMap.get(CRServo.class, servoName);
        SpindyCS = strategy;
    }

    public class AutonomousSpindy extends AutonomousComponentBehaviors {
        // CALIBRATION: Measure how long it takes for ONE FULL ROTATION (360°) at full power
        // Run the servo at setPower(1) and time it with a stopwatch
        private static final long FULL_ROTATION_MS = 1000;  // CHANGE THIS TO YOUR ACTUAL TIME

        /**
         * Rotate exactly 120 degrees
         * 120° / 360° = 1/3 of full rotation
         */
        public void rotateTo120Degrees() {
            long timeNeeded = (long) (FULL_ROTATION_MS * (120.0 / 360.0));
            spindy.setPower(1);
            ThreadExtensions.TrySleep(timeNeeded);
            spindy.setPower(0);
        }

        /**
         * Rotate exactly 60 degrees
         * 60° / 360° = 1/6 of full rotation
         */
        public void rotateTo60Degrees() {
            long timeNeeded = (long) (FULL_ROTATION_MS * (60.0 / 360.0));
            spindy.setPower(1);
            ThreadExtensions.TrySleep(timeNeeded);
            spindy.setPower(0);
        }

        public void SpindyPos(){spindy.setPower(1);}
        public void SpindyStop(){spindy.setPower(0);}
        public void SpindyNeg(){spindy.setPower(-1);}
    }

    @Override
    public AutonomousSpindy getAutonomousBehaviors() {return auton;}

    private final CRServo spindy;
    private final AutonomousSpindy auton = new AutonomousSpindy();
    private final SoSpinSerControlStrategy SpindyCS;

    @Override
    public void move(Gamepad gamepad) {
        SpindyCS.SCS(this, gamepad);
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("Spindy Power", spindy.getPower());
    }
}
