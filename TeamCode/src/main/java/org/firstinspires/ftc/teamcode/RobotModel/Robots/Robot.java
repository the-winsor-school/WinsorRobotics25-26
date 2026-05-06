package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.MechAssembly;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot
{

    protected interface IRobotStrategy { }
    protected Robot.IRobotStrategy strategy;

    public abstract class AutonomousRobot
    {
        protected final Telemetry telemetry;

        /**
         * Requires that any extending class provide matching parameter types in its constructor.
         * @param driveTrain Any AutonomousDriving implementation
         * @param mechAssembly Any AutonomousMechBehaviors implementation
         * @param telemetry Telemetry injected at construction time
         */
        public AutonomousRobot(
                DriveTrain.AutonomousDriving driveTrain,
                MechAssembly.AutonomousMechBehaviors mechAssembly,
                Telemetry telemetry)
        {
            this.telemetry = telemetry;
        }

        public void reportStatus(String status) { telemetry.addLine(status); }
        public void reportData(String key, Object value) { telemetry.addData(key, value); }
        public void clearTelemetry() { telemetry.clear(); }
    }

    protected interface IControlStrategy {  }

    /**
     * This abstract method definition tells inheriting classes that they MUST define a
     * way to run Autonomously.
     * @return a CONCRETE implementation of the ABSTRACT AutonomousRobot type.
     * @param <T> Any type which extends AutonomousRobot
     */
    public abstract <T extends AutonomousRobot> T getAutonomousRobot();

    protected VisionPortal visionPortal;
    protected DriveTrain driveTrain;
    protected MechAssembly mechAssembly;
    protected Telemetry telemetry;

    protected Robot(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    /**
     * Propagates the owned telemetry reference to subsystems.
     * Call this in subclass constructors after driveTrain and mechAssembly are assigned.
     */
    protected void initializeSubsystems()
    {
        if (driveTrain != null) driveTrain.initializeTelemetry(telemetry);
        if (mechAssembly != null) mechAssembly.initializeTelemetry(telemetry);
    }

    /**
     * Single flush point for all telemetry — the only place that calls telemetry.update().
     */
    public void updateTelemetry() {
        if (driveTrain != null) driveTrain.updateTelemetry();
        if (mechAssembly != null) mechAssembly.updateTelemetry();
        telemetry.update();
    }

    /**
     * update the robot state!  Pass along the gamepads to the different modules.
     * @param gamepad1 driver gamepad
     * @param gamepad2 mech gamepad
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (driveTrain != null) driveTrain.drive(gamepad1);
        if (mechAssembly != null) mechAssembly.giveInstructions(gamepad2);
    }
}
