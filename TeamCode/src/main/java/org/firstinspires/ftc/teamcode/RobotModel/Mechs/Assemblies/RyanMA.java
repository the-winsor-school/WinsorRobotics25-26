package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.RyanIntake;

public class RyanMA extends MechAssembly {

    // this is the line that you were missing here~
    // It would also be completely valid to change RyanIntake (the type) to just be a SpinnyIntake
    // no need to re-invent the wheel every time >.<
    private final RyanIntake ryanintake;

    public RyanMA(HardwareMap hardwareMap) {
        // gotta declare this variable~
        ryanintake = new RyanIntake(hardwareMap, "ryanmotor",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger);
                });
        // fix this error by passing down ryanintake.getAutonomousBehaviors() in the parenthesis
        auton = new AutonomousRyanMA();
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

    // that crazy T asjddjkfhal stuff just gets replaced with the Autonomous object~
    @Override
    public AutonomousRyanMA getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        //gamepad to move components/motors and stuff
        // our job here is to pass along instructions to each of the components of the Mech Assembly
        // So you should invoke ryanintake.move(...) here~
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        // Similarly, we need to pass along Telemetry requests to each of the components
        // so invoke ryanintake.updateTelemetry(...) here~
    }
}
