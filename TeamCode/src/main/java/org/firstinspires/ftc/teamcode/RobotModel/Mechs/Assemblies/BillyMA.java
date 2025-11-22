package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

    }

    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return null;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
