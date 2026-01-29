package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheel;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaSpindexServo;

public class SodapopMA extends MechAssembly{
    public SodapopMA(HardwareMap hardwareMap) {
        //color sensor???
        //not sure how to code the cam yet
        //lift (2)
        sodamouth = new SodaIntake(hardwareMap, "soinmotor",
                //controls
                );
        sodaflywheel = new SodaFlywheel(hardwareMap, "soflymotor",
                //controls
                );
        sodaspindex = new SodaSpindexServo(hardwareMap, "sospinservo",
                //controls
                );



        //insert parameters for the line below
        auton = new AutonomousSodapopMA();
    }

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        //parameters/behaviors for each component
        private final SodaIntake.AutonomousSodaIntake AutonSOIN;
        private final SodaFlywheel.AutonomousSoFly AutonSOFLY;
        private final SodaSpindexServo.AutonomousSpindy AutonSOSS;

        public SodapopMA.AutonomousSodapopMA(
                SodaSpindexServo.AutonomousSpindy autonSOIN,
                SodaFlywheel.AutonomousSoFly autonSOFLY,
                SodaSpindexServo.AutonomousSpindy autonSOSS) {
            AutonSOIN = autonSOIN;
            AutonSOFLY = autonSOFLY;
            AutonSOSS = autonSOSS;

        }
    }

    private final AutonomousSodapopMA auton;
    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {return null;}

    private final SodaIntake sodamouth;
    private final SodaFlywheel sodaflywheel;
    private final SodaSpindexServo sodaspindex;
    @Override
    public void giveInstructions(Gamepad gamepad) {
        sodamouth.move(gamepad);
        sodaflywheel.move(gamepad);
        sodaspindex.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        //telemetry if needed
    }
}
