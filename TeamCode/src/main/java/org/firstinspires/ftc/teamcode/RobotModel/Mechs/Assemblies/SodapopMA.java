package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.FlappyServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheel;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;

public class SodapopMA extends MechAssembly{

    public SodapopMA(HardwareMap hardwareMap) {
        //spindex (servo)
        //color sensor???
        //lift (2)

        sodamouth = new SodaIntake(hardwareMap, "soinmotor",
                //controls - you'll need to fill these in based on your SodaIntake interface
                (motor, gamepad) -> {
                    // Add your control strategy here
                    // Example: motor.setPower(gamepad.right_trigger);
                }
        );

        sodaflywheel = new SodaFlywheel(hardwareMap, "soflymotor",
                //controls - you'll need to fill these in based on your SodaFlywheel interface
                (motor, gamepad) -> {
                    // Add your control strategy here
                    // Example: motor.setPower(gamepad.left_trigger);
                }
        );

        // Initialize the LimelightForSoda component
        limelightForSoda = new LimelightForSoda(hardwareMap, "limelight",
                (gamepad) -> {
                    // TeleOp control strategy for Limelight
                    // You can add gamepad controls here if needed
                    // For example, switching pipelines:
                    // if (gamepad.dpad_up) { /* switch to AprilTag pipeline */ }
                    // if (gamepad.dpad_down) { /* switch to color detection pipeline */ }
                });

        // Initialize autonomous behaviors with all components
        auton = new AutonomousSodapopMA(
                sodamouth.getAutonomousBehaviors(),
                sodaflywheel.getAutonomousBehaviors(),
                limelightForSoda.getAutonomousBehaviors()
        );
    }

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        // Autonomous behaviors for each component
        public final SodaIntake.AutonomousSodaIntake AutonSOIN;
        public final SodaFlywheel.AutonomousSodaFlywheel AutonSOFLY;
        public final LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightForSoda;

        public AutonomousSodapopMA(SodaIntake.AutonomousSodaIntake autonSOIN,
                                   SodaFlywheel.AutonomousSodaFlywheel autonSOFLY,
                                   LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightForSoda) {
            this.AutonSOIN = autonSOIN;
            this.AutonSOFLY = autonSOFLY;
            this.limelightForSoda = limelightForSoda;
        }
    }

    private final AutonomousSodapopMA auton;

    @Override
    public AutonomousSodapopMA getAutonomousBehaviors() {
        return auton;
    }

    private final SodaIntake sodamouth;
    private final SodaFlywheel sodaflywheel;
    public final LimelightForSoda limelightForSoda; // Made public so SodapopRobot can access it

    @Override
    public void giveInstructions(Gamepad gamepad) {
        sodamouth.sodamove(gamepad);
        sodaflywheel.move(gamepad);
        limelightForSoda.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        // Add telemetry from all components
        limelightForSoda.update(telemetry);
        // Add other component telemetry as needed
        // sodamouth.update(telemetry);
        // sodaflywheel.update(telemetry);
    }
}
