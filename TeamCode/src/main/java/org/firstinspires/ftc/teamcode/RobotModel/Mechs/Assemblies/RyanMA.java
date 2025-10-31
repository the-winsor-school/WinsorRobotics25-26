package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.BallDetectionComponent;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.RyanIntake;

public class RyanMA extends MechAssembly {

    private final RyanIntake ryanintake;
    private final BallDetectionComponent balldetector;
    public RyanMA(HardwareMap hardwareMap) {
        // gotta declare this variable~
        ryanintake = new RyanIntake(hardwareMap, "ryanmotor",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger);
                });
        auton = new AutonomousRyanMA(ryanintake.getAutonomousBehaviors());
        balldetector = new BallDetectionComponent(hardwareMap, "Webcam 1",
                (purpleProcessor, greenProcessor, gamepad) -> {});
    }

    public class AutonomousRyanMA extends AutonomousMechBehaviors
    {
        // Here we need to have the Autonomous parts of each of the components,
        // so this is the place to have an AutonomousRyanIntake property
        public final RyanIntake.AutonomousRyanIntake autonomousRyanIntake;

        // we also have to have it initialized in the Constructor method~
        public AutonomousRyanMA(RyanIntake.AutonomousRyanIntake autonomousRyanIntake) {
            this.autonomousRyanIntake = autonomousRyanIntake;
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
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        balldetector.update(telemetry);
    }
}
