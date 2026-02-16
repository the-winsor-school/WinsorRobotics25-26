/* RobotModel/Robots/SodaRobot.java */
package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;
import org.firstinspires.ftc.teamcode.RobotModel.Odometry.OdometryTracker;

public class SodapopRobot extends Robot {

    public class AutonomousSodapopRobot extends AutonomousRobot {
        /**
         * Public access to autonomous drive train and mech assembly
         */
        public final MecanumDrive.AutonomousMecanumDrive driveTrain;
        public final SodapopMA.AutonomousSodapopMA mechAssembly;
        public final OdometryTracker odometry;

        public AutonomousSodapopRobot(
                MecanumDrive.AutonomousMecanumDrive driveTrain,
                SodapopMA.AutonomousSodapopMA mechAssembly,
                OdometryTracker odometry) {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
            this.odometry = odometry;
        }
    }

    private final AutonomousSodapopRobot auton;
    private final OdometryTracker odometry;

    @Override
    public AutonomousSodapopRobot getAutonomousRobot() {
        return auton;
    }

    public SodapopRobot(HardwareMap hardwareMap) {
        // ===== MECANUM DRIVE TRAIN =====
        driveTrain = new MecanumDrive(
                hardwareMap,
                new MecanumDrive.OrientationConfiguration(
                        DcMotorSimple.Direction.REVERSE,  // LB
                        DcMotorSimple.Direction.FORWARD,  // LF
                        DcMotorSimple.Direction.FORWARD,  // RB
                        DcMotorSimple.Direction.REVERSE   // RF
                )
        );

        // ===== ODOMETRY =====
        // Update these names to match your hardware configuration
        odometry = new OdometryTracker(hardwareMap, "lb", "rf", "lf");

        // ===== MECH ASSEMBLY =====
        mechAssembly = new SodapopMA(hardwareMap);

        // ===== CREATE AUTONOMOUS ROBOT =====
        auton = new AutonomousSodapopRobot(
                driveTrain.getAutonomousDriving(),
                mechAssembly.getAutonomousBehaviors(),
                odometry
        );
    }

    @Override
    public void update(com.qualcomm.robotcore.hardware.Gamepad gamepad1, com.qualcomm.robotcore.hardware.Gamepad gamepad2) {
        // Update odometry every loop
        odometry.update();
        super.update(gamepad1, gamepad2);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        odometry.updateTelemetry(telemetry);
    }
}
