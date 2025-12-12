package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.FlappyServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Shooter;


/**
 * TODO:  This is a good example of a thing that we need in this project
 *        An "EmptyMechAssembly" which just ensures that nothing throws
 *        a NullReferenceException when being accessed.  (currently this
 *        still does have a null in it....)  But, otherwise, this is the
 *        EmptyMechAssembly
 */
public class BillyMA extends MechAssembly
{
    public BillyMA(HardwareMap hardwareMap) {

        canon = new Shooter(hardwareMap, "BANGBANGBANG",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger*-1);
                });;

        auton = new AutonomousBillyMA(canon.getAutonomousBehaviors());
    }

    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return null;
    }


    public class AutonomousBillyMA extends AutonomousMechBehaviors {

        private final Shooter.AutonomousShooterBehavior AutonShooter;
        public AutonomousBillyMA(Shooter.AutonomousShooterBehavior autonShooter) {
            AutonShooter = autonShooter;
        }
    }

    private final BillyMA.AutonomousBillyMA auton;

    private final Shooter canon;

    @Override
    public void giveInstructions(Gamepad gamepad) {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
