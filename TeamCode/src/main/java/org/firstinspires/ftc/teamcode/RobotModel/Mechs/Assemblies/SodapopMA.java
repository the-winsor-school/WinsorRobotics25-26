package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightVision;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheel;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaLift;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaSpindexServo;

public class SodapopMA extends MechAssembly{
    public SodapopMA(HardwareMap hardwareMap) {
        //color sensor are a need ???
        sodamouth = new SodaIntake(hardwareMap, "soinmotor",
                (motor, gamepad) -> {
                    if (gamepad.right_trigger!=0){
                        motor.setPower(gamepad.right_trigger); //Tweak if we want a set intake speed
                    }
                    else {
                        motor.setPower(gamepad.left_trigger);
                    }
                }
                );
        sodaflywheel = new SodaFlywheel(hardwareMap, "soflymotor",
                (motor, gamepad) -> {
                    if (gamepad.right_bumper){
                        motor.setPower(0.75); //Tweak if we want an adaptive shooter flywheel speed or if it is too slow/fast
                    }
                    else if (gamepad.left_bumper){
                        motor.setPower(-1); //In case of jamming
                    }
                }
                //(based on limelight???)
        );
        sodaspindex = new SodaSpindexServo(hardwareMap, "sospinservo",
                (servo, gamepad) -> {
                    if (gamepad.x){
                        ((SodaSpindexServo.AutonomousSpindy)
                                SodapopMA.this.sodaspindex.getAutonomousBehaviors()).rotateTo120Degrees();  //Tweak if we want an adaptive shooter flywheel speed or if it is too slow/fast
                    }
                }
        );
        //sodalift = new SodaLift(hardwareMap, "solift",
                //controls
        //);
        limelightVision = new LimelightVision(hardwareMap, "limelight",
                gamepad -> {
                    // Add Limelight control strategy here
                }
        );

        //insert parameters for the line below
        auton = new AutonomousSodapopMA(
                sodamouth.getAutonomousBehaviors(),
                sodaflywheel.getAutonomousBehaviors(),
                sodaspindex.getAutonomousBehaviors(),
                //sodalift.getAutonomousBehaviors(),
                limelightVision.getAutonomousBehaviors()
        );
    }

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        //parameters/behaviors for each component
        private final SodaIntake.AutonomousSodaIntake AutonSOIN;
        private final SodaFlywheel.AutonomousSoFly AutonSOFLY;
        private final SodaSpindexServo.AutonomousSpindy AutonSOSS;
//        private final SodaLift.AutonomousSodaLift AutonSOLIFT;
        private final LimelightVision.AutonomousLimelightVision AutonLIMELIGHT;

        public AutonomousSodapopMA(
        SodaIntake.AutonomousSodaIntake autonSOIN,
        SodaFlywheel.AutonomousSoFly autonSOFLY,
        SodaSpindexServo.AutonomousSpindy autonSOSS,
     //   SodaLift.AutonomousSodaLift autonSOLIFT,
        LimelightVision.AutonomousLimelightVision autonLIMELIGHT) {
            AutonSOIN = autonSOIN;
            AutonSOFLY = autonSOFLY;
            AutonSOSS = autonSOSS;
    //        AutonSOLIFT = autonSOLIFT;
            AutonLIMELIGHT = autonLIMELIGHT;
        }
    }

    private final AutonomousSodapopMA auton;
    @Override
    public AutonomousSodapopMA getAutonomousBehaviors() {return auton;}

    private final SodaIntake sodamouth;
    private final SodaFlywheel sodaflywheel;
    private final SodaSpindexServo sodaspindex;
    //private final SodaLift sodalift;
    private final LimelightVision limelightVision;

    @Override
    public void giveInstructions(Gamepad gamepad) {
        sodamouth.move(gamepad);
        sodaflywheel.move(gamepad);
        sodaspindex.move(gamepad);
    //    sodalift.move(gamepad);
        limelightVision.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        limelightVision.update(telemetry);
    }
}
