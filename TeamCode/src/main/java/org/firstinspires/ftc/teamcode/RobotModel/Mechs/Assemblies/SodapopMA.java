package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.AimingAssistant;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.BallDetectionComponent;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheel;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaLift;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaSpindexServo;

public class SodapopMA extends MechAssembly {

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        public final SodaIntake.AutonomousSodaIntake AutonSOIN;
        public final SodaFlywheel.AutonomousSoFly AutonSOFLY;
        public final SodaSpindexServo.AutonomousSpindy AutonSOSS;
        public final LimelightForSoda.AutonomousLimelightForSodaBehaviors AutonLIMELIGHT;

        public AutonomousSodapopMA(
                SodaIntake.AutonomousSodaIntake autonSOIN,
                SodaFlywheel.AutonomousSoFly autonSOFLY,
                SodaSpindexServo.AutonomousSpindy autonSOSS,
                LimelightForSoda.AutonomousLimelightForSodaBehaviors autonLIMELIGHT) {
            AutonSOIN = autonSOIN;
            AutonSOFLY = autonSOFLY;
            AutonSOSS = autonSOSS;
            AutonLIMELIGHT = autonLIMELIGHT;
        }
    }

    private final AutonomousSodapopMA auton;
    private final SodaIntake sodamouth;
    private final SodaFlywheel sodaflywheel;
    private final SodaSpindexServo sodaspindex;
    private final LimelightForSoda limelightForSoda;
    private final BallDetectionComponent balldetector;
    private final AimingAssistant aimingAssistant;

    public SodapopMA(HardwareMap hardwareMap) {
        // Intake
        sodamouth = new SodaIntake(hardwareMap, "soinmotor",
                (motor, gamepad) -> {
                    if (gamepad.right_trigger != 0) {
                        motor.setPower(gamepad.right_trigger);
                    } else {
                        motor.setPower( -1 * gamepad.left_trigger);
                    }
                });

        // Flywheel
        sodaflywheel = new SodaFlywheel(hardwareMap, "soflymotor",
                (motor, gamepad) -> {
                    if (gamepad.right_bumper) {
                        motor.setVelocity(2240);
                    } else if (gamepad.left_bumper) {
                        motor.setPower(-1120);
                    }
                });

        // Spindex Servo
        sodaspindex = new SodaSpindexServo(hardwareMap, "sospinservo",
                (servo, gamepad) -> {
                    if (gamepad.dpad_right) {
                        ((SodaSpindexServo.AutonomousSpindy)
                                SodapopMA.this.sodaspindex.getAutonomousBehaviors()).rotateTo120DegreesRight();
                    }
                    if (gamepad.dpad_left) {
                        ((SodaSpindexServo.AutonomousSpindy)
                                SodapopMA.this.sodaspindex.getAutonomousBehaviors()).rotateTo120DegreesLeft();
                    }
                });

        // Limelight Vision (UPDATED: use LimelightForSoda with HTTP API)
        // Replace "192.168.1.100" with our actual Limelight IP
        // Or use "limelight.local" our network supports mDNS
        limelightForSoda = new LimelightForSoda(hardwareMap, "192.168.1.100",
                (gamepad) -> {
                    // Limelight reads data passively via HTTP
                });

        // Aiming Assistant
        aimingAssistant = new AimingAssistant(
                limelightForSoda.getAutonomousBehaviors());  // Pass Limelight behaviors


        // Ball Detection
        balldetector = new BallDetectionComponent(hardwareMap, "Webcam 1",
                (purpleProcessor, greenProcessor, gamepad) -> {});

        // Initialize Autonomous behaviors
        auton = new AutonomousSodapopMA(
                sodamouth.getAutonomousBehaviors(),
                sodaflywheel.getAutonomousBehaviors(),
                sodaspindex.getAutonomousBehaviors(),
                limelightForSoda.getAutonomousBehaviors()
        );
    }

    @Override
    public AutonomousSodapopMA getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        // Update aiming mode based on gamepad input
        aimingAssistant.updateAimingMode(gamepad);

        sodamouth.move(gamepad);
        sodaflywheel.move(gamepad);
        sodaspindex.move(gamepad);
        limelightForSoda.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        limelightForSoda.update(telemetry);
        balldetector.update(telemetry);
    }

    /**
     * Get the aiming assistant for manual control
     */
    public AimingAssistant getAimingAssistant() {
        return aimingAssistant;
    }

    /**
     * Get Limelight autonomous behaviors for autonomous strategies
     */
    public LimelightForSoda.AutonomousLimelightForSodaBehaviors getLimelightBehaviors() {
        return auton.AutonLIMELIGHT;
    }

    /**
     * Get Limelight component for direct access if needed
     */
    public LimelightForSoda getLimelightComponent() {
        return limelightForSoda;
    }
}
