/* RobotModel/Mechs/Assemblies/SodapopMA.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.GamepadExtensions;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaColorSensorArray;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheelWithPID;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaSpindexerWithEncoder;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaTurretAdjuster;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaTurretTurner;

public class SodapopMA extends MechAssembly {

    // ===== COMPONENT DECLARATIONS =====
    private final SodaIntake sodaIntake;
    private final SodaSpindexerWithEncoder sodaSpindexer;
    private final SodaColorSensorArray sodaColorSensors;
    private final SodaFlywheelWithPID sodaFlywheel;
    private final SodaTurretTurner sodaTurretTurner;
    private final SodaTurretAdjuster sodaHoodAdjuster;
    private final LimelightForSoda sodaLimelight;
    private final AutonomousSodapopMA auton;

    public SodapopMA(HardwareMap hardwareMap) {

        // ===== INTAKE SYSTEM =====
        sodaIntake = new SodaIntake(hardwareMap, "intakeMotor",
                (motor, gamepad) -> {
                    // Right trigger = intake in
                    if (gamepad.right_trigger > 0.1) {
                        motor.setPower(gamepad.right_trigger);
                    }
                    // Left trigger = intake out (reverse, for jamming)
                    else if (gamepad.left_trigger > 0.1) {
                        motor.setPower(-gamepad.left_trigger);
                    }
                    // Otherwise stop
                    else {
                        motor.setPower(0);
                    }
                }
        );

        // ===== SPINDEXER WITH ENCODER =====
        // Simple control: just control servo power directly, no autonomous calls
        sodaSpindexer = new SodaSpindexerWithEncoder(hardwareMap, "spindexerServo", "encoderMotor",
                (servo, encoder, gamepad) -> {
                    // X button = rotate forward (shoot 1)
                    if (gamepad.x) {
                        servo.setPower(1.0);
                        ThreadExtensions.TrySleep(667);  // 1/3 of rotation time
                        servo.setPower(0);
                    }
                    // B button = rotate forward 3 times (shoot 3)
                    else if (gamepad.b) {
                        for (int i = 0; i < 3; i++) {
                            servo.setPower(1.0);
                            ThreadExtensions.TrySleep(667);
                            servo.setPower(0);
                            ThreadExtensions.TrySleep(300);
                        }
                    }
                    // Left arrow = rotate backwards
                    else if (gamepad.dpad_left) {
                        servo.setPower(-1.0);
                        ThreadExtensions.TrySleep(667);
                        servo.setPower(0);
                    }
                    // Right arrow = rotate forwards
                    else if (gamepad.dpad_right) {
                        servo.setPower(1.0);
                        ThreadExtensions.TrySleep(667);
                        servo.setPower(0);
                    }
                    // Otherwise stop
                    else {
                        servo.setPower(0);
                    }
                }
        );

        // ===== COLOR SENSOR ARRAY =====
        sodaColorSensors = new SodaColorSensorArray(hardwareMap,
                new String[]{"colorSensor0", "colorSensor1", "colorSensor2"},
                (sensors, gamepad) -> {
                    // Color sensors are read-only in teleop
                }
        );

        // ===== FLYWHEEL WITH VELOCITY PID =====
        sodaFlywheel = new SodaFlywheelWithPID(hardwareMap, "flywheelMotor",
                (motor, flywheelComponent, gamepad) -> {
                    // Right bumper = shoot (flywheel at full power)
                    if (gamepad.right_bumper) {
                        motor.setPower(0.8);
                    }
                    // Left bumper = reverse (unjam)
                    else if (gamepad.left_bumper) {
                        motor.setPower(-0.5);
                    }
                    // Otherwise stop
                    else {
                        motor.setPower(0);
                    }
                }
        );

        // ===== TURRET TURNER (CONTINUOUS ROTATION SERVO) =====
        sodaTurretTurner = new SodaTurretTurner(hardwareMap, "turretServo",
                (servo, gamepad) -> {
                    float rightStickX = GamepadExtensions.GetRightStickX(gamepad);
                    servo.setPower(rightStickX);
                }
        );

        // ===== HOOD ADJUSTER (STANDARD SERVO) =====
        sodaHoodAdjuster = new SodaTurretAdjuster(hardwareMap, "hoodServo",
                (servo, gamepad) -> {
                    float rightStickY = GamepadExtensions.GetRightStickY(gamepad);
                    double position = (rightStickY + 1.0) / 2.0;
                    servo.setPosition(Math.max(0, Math.min(1, position)));
                }
        );

        // ===== LIMELIGHT VISION =====
        sodaLimelight = new LimelightForSoda(hardwareMap, "limelight",
                (gamepad) -> {
                    // Limelight is read-only in teleop
                }
        );

        // ===== CREATE AUTONOMOUS BEHAVIORS =====
        auton = new AutonomousSodapopMA(
                sodaIntake.getAutonomousBehaviors(),
                sodaSpindexer.getAutonomousBehaviors(),
                sodaColorSensors.getAutonomousBehaviors(),
                sodaFlywheel.getAutonomousBehaviors(),
                sodaTurretTurner.getAutonomousBehaviors(),
                sodaHoodAdjuster.getAutonomousBehaviors(),
                sodaLimelight.getAutonomousBehaviors()
        );
    }

    // ===== AUTONOMOUS BEHAVIORS CLASS =====
    public class AutonomousSodapopMA extends AutonomousMechBehaviors {

        public final SodaIntake.AutonomousSodaIntake sodaIntake;
        public final SodaSpindexerWithEncoder.AutonomousSpindexer sodaSpindexer;
        public final SodaColorSensorArray.AutonomousColorSensors sodaColorSensors;
        public final SodaFlywheelWithPID.AutonomousFlywheel sodaFlywheel;
        public final SodaTurretTurner.AutonomousTurretTurner sodaTurretTurner;
        public final SodaTurretAdjuster.AutonomousHoodAdjuster sodaHoodAdjuster;
        public final LimelightForSoda.AutonomousLimelightForSodaBehaviors sodaLimelight;

        public AutonomousSodapopMA(
                SodaIntake.AutonomousSodaIntake sodaIntake,
                SodaSpindexerWithEncoder.AutonomousSpindexer sodaSpindexer,
                SodaColorSensorArray.AutonomousColorSensors sodaColorSensors,
                SodaFlywheelWithPID.AutonomousFlywheel sodaFlywheel,
                SodaTurretTurner.AutonomousTurretTurner sodaTurretTurner,
                SodaTurretAdjuster.AutonomousHoodAdjuster sodaHoodAdjuster,
                LimelightForSoda.AutonomousLimelightForSodaBehaviors sodaLimelight) {
            this.sodaIntake = sodaIntake;
            this.sodaSpindexer = sodaSpindexer;
            this.sodaColorSensors = sodaColorSensors;
            this.sodaFlywheel = sodaFlywheel;
            this.sodaTurretTurner = sodaTurretTurner;
            this.sodaHoodAdjuster = sodaHoodAdjuster;
            this.sodaLimelight = sodaLimelight;
        }
    }

    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return (T) auton;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        sodaIntake.move(gamepad);
        sodaSpindexer.move(gamepad);
        sodaFlywheel.move(gamepad);
        sodaTurretTurner.move(gamepad);
        sodaHoodAdjuster.move(gamepad);
        sodaLimelight.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== SODAPOP MECH ASSEMBLY =====");
        sodaIntake.update(telemetry);
        sodaSpindexer.update(telemetry);
        sodaColorSensors.update(telemetry);
        sodaFlywheel.update(telemetry);
        sodaTurretTurner.update(telemetry);
        sodaHoodAdjuster.update(telemetry);
        sodaLimelight.update(telemetry);
        telemetry.addLine("==================================");
    }
}
