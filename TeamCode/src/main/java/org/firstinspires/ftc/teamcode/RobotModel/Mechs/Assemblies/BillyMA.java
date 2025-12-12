package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.GamepadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.BallDetectionComponent;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.MechComponent;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Shooter;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;

public class BillyMA extends MechAssembly {
    private final SpinnyIntake intake;

    private final Shooter shooter;
    public BillyMA(HardwareMap hardwareMap) {
        // gotta declare this variable~
        intake = new SpinnyIntake(hardwareMap, "billymotor",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger);
                });
        shooter = new Shooter(hardwareMap,
                "doubleshooterleft", "doubleshooterright",
                ( motor, gamepad )-> {
                    if (gamepad.b) {
                        left.setPower(-1);
                        right.setPower(-1);
                    }
                    else{
                        left.setPower(0);
                        right.setPower(0);
                    }
                });
        auton = new AutonomousRyanMA(intake.getAutonomousBehaviors(),
                shooter.getAutonomousBehaviors());
    }

    public class AutonomousRyanMA extends AutonomousMechBehaviors
    {
        // Here we need to have the Autonomous parts of each of the components,
        // so this is the place to have an AutonomousRyanIntake property
        public final SpinnyIntake.AutonomousIntakeBehaviors autonIntake;
        public final Shooter.AutonomousShooterBehavior autonShooter;

        // we also have to have it initialized in the Constructor method~
        public AutonomousRyanMA(SpinnyIntake.AutonomousIntakeBehaviors autonIntake,
                                Shooter.AutonomousShooterBehavior autonShooter) {
            this.autonIntake = autonIntake;
            this.autonShooter = autonShooter;
        }
    }

    private final AutonomousRyanMA auton;

    @Override
    public AutonomousRyanMA getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        // our job here is to pass along instructions to each of the components of the Mech Assembly
        // So you should invoke ryanintake.move(...) here~
        intake.move(gamepad);
        shooter.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        //balldetector.update(telemetry);
    }
}
