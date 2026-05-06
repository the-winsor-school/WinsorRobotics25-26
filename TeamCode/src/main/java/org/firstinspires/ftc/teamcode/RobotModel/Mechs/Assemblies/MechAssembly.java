package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mech Assembly is a Composite of all the mech components
 * in a robot.  Inheritors should have discrete Mech Component
 * implementations that it hands down instructions to.
 */
public abstract class MechAssembly
{
    protected interface IAssemblyStrategy { }
    protected MechAssembly.IAssemblyStrategy strategy;

    public abstract class AutonomousMechBehaviors {
        protected final Telemetry telemetry;
        public AutonomousMechBehaviors(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        public void reportStatus(String status) { telemetry.addLine(status); }
        public void reportData(String key, Object value) { telemetry.addData(key, value); }
    }
    public abstract <T extends AutonomousMechBehaviors> T getAutonomousBehaviors();

    /**
     * Pass along gamepad instructions to subcomponents.
     * @param gamepad the Gamepad
     */
    public abstract void giveInstructions(Gamepad gamepad);

    protected Telemetry telemetry;

    public abstract void initializeTelemetry(Telemetry telemetry);
    public abstract void updateTelemetry();
}
