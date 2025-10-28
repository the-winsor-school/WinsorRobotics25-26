package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.RyanIntake;

public class RyanMA extends MechAssembly {
    public RyanMA(HardwareMap hardwareMap) {
        ryanintake = new RyanIntake(hardwareMap, "ryanmotor",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger);
                });
        auton = new AutonomousRyanMA();
    }

    public class AutonomousRyanMA extends AutonomousMechBehaviors {

    }

    private final AutonomousRyanMA auton;
    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        //gamepad to move components/motors and stuff
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
