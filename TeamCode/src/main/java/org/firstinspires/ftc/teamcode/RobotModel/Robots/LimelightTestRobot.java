package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions.LimelightPosePoller;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.NoDriveTrain;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.NoMechAssembly;

/**
 * Minimal robot used to exercise LimelightPosePoller without drivetrain or mechanism hardware.
 */
public class LimelightTestRobot extends Robot
{
    private final LimelightPosePoller limelight;

    public class AutonomousLimelightTestRobot extends AutonomousRobot
    {
        public final NoDriveTrain.NoAutonomousDriving driveTrain;
        public final NoMechAssembly.NoAutonomousMechBehaviors mechAssembly;
        public final LimelightPosePoller limelight;

        public AutonomousLimelightTestRobot(
                NoDriveTrain.NoAutonomousDriving driveTrain,
                NoMechAssembly.NoAutonomousMechBehaviors mechAssembly,
                LimelightPosePoller limelight)
        {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
            this.limelight = limelight;
        }
    }

    private final AutonomousLimelightTestRobot auton;

    @Override
    public AutonomousLimelightTestRobot getAutonomousRobot()
    {
        return auton;
    }

    public LimelightTestRobot(LimelightPosePoller limelight)
    {
        // Provide no-op components so callers avoid null checks.
        this.driveTrain = new NoDriveTrain();
        this.mechAssembly = new NoMechAssembly();
        this.visionPortal = null; // not used in this test robot
        this.limelight = limelight;

        // Build the autonomous view that holds the injected Limelight poller.
        this.auton = new AutonomousLimelightTestRobot(
                ((NoDriveTrain) this.driveTrain).getAutonomousDriving(),
                ((NoMechAssembly) this.mechAssembly).getAutonomousBehaviors(),
                limelight
        );
    }

    @Override
    public void updateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry)
    {
        // Keep existing drive/mech telemetry (no-ops here) for parity with other robots.
        driveTrain.updateTelemetry(telemetry);
        mechAssembly.updateTelemetry(telemetry);

        if (limelight != null)
        {
            java.util.List<org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions.AprilTagPoseData> tags =
                    limelight.getLatest();
            telemetry.addData("Limelight tag count", tags.size());
            int idx = 0;
            for (org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions.AprilTagPoseData tag : tags)
            {
                String poseStr = (tag.t6t_rs != null)
                        ? String.format("x=%.2f y=%.2f z=%.2f yaw=%.1f pitch=%.1f roll=%.1f",
                        tag.t6t_rs.x, tag.t6t_rs.y, tag.t6t_rs.z,
                        tag.t6t_rs.yaw, tag.t6t_rs.pitch, tag.t6t_rs.roll)
                        : "pose unavailable";
                telemetry.addData("Tag " + idx,
                        "id=%d fam=%s ta=%.3f %s", tag.fID, tag.fam, tag.ta, poseStr);
                idx++;
            }
        }

        telemetry.update();
    }
}
