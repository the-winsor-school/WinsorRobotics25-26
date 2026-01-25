package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodapopMA extends MechAssembly{
    public SodapopMA(HardwareMap hardwareMap) {
        /*
         * TODO:
         * Add components such as:
         * top intake, bottom intake (intake mech isnt finalized so idk what motor/how many motors for this component)
         * spindex
         * color sensor???
         * not sure how to code the cam yet
         * shooter/turret
         * lift??
         */
        //insert parameters for the line below
        auton = new AutonomousSodapopMA();
    }

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        //parameters/behaviors for each component
    }

    private final AutonomousSodapopMA auton;
    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {return null;}
    //private final component thingas
    @Override
    public void giveInstructions(Gamepad gamepad) {
        //controls and stuff
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        //telemetry if needed
    }
}
