# LimelightPosePoller Injection Guide

This guide shows how to inject `LimelightPosePoller` into an `AutonomousRobot` class, following the same pattern used for traditional vision setup in `FranklinRobot` / `FranklinAuton`.

## Goal

Create the poller in the OpMode (where `opModeIsActive()` is available), then pass it into the autonomous robot so your auton strategies can read the latest Limelight fiducials.

## Suggested injection pattern

### 1) Add the poller to the autonomous robot type

Example for `FranklinRobot.AutonomousFranklinRobot`:

```java
public class AutonomousFranklinRobot extends AutonomousRobot {
    public final StandardTankDrive.AutonomousTankDrive driveTrain;
    public final FranklinMA.AutonomousFranklinMA mechAssembly;
    public final LimelightPosePoller limelight;

    public AutonomousFranklinRobot(
            StandardTankDrive.AutonomousTankDrive driveTrain,
            FranklinMA.AutonomousFranklinMA mechAssembly,
            LimelightPosePoller limelight) {
        super(driveTrain, mechAssembly);
        this.driveTrain = driveTrain;
        this.mechAssembly = mechAssembly;
        this.limelight = limelight;
    }
}
```

### 2) Construct the poller in the OpMode and inject it

Because `LimelightPosePoller` needs an `OpMode`, build it in your OpMode (like `FranklinAuton`) and pass it in:

```java
LimelightResultsClient client = new LimelightResultsClient();
LimelightPosePoller poller = new LimelightPosePoller(this, client, 100);

// Add a FranklinRobot constructor that accepts the poller and injects it.
robot = new FranklinRobot(hardwareMap, poller);
AutonomousFranklinRobot auton = robot.getAutonomousRobot();
```

If you prefer to keep `getAutonomousRobot()` as the factory, add an overload to your robot (or a setter) so the poller can be provided before the instance is returned.

### 3) Start polling after `waitForStart()`

`LimelightPosePoller` exits if `opModeIsActive()` is false, so start it after the OpMode begins:

```java
waitForStart();
poller.start();
autonStrategy.execute();
```

### 4) Read the latest fiducials inside auton logic

Anywhere you have the autonomous robot instance:

```java
List<AprilTagPoseData> tags = auton.limelight.getLatest();
```

`getLatest()` is thread-safe and returns the most recent snapshot.

## Notes

- The poller should be stopped when your OpMode ends (`poller.stop()` or `poller.close()`).
- If you start the poller before `waitForStart()`, it will immediately exit because `opModeIsActive()` is false.
- For consistency with existing vision usage in `FranklinRobot`, keep the poller as a dependency owned by the autonomous robot, and let auton strategies read from it.

## Example: Find a specific AprilTag (Tag ID 24) and report pose

This pattern searches the latest fiducials for a target ID and prints its pose to telemetry.

```java
int targetId = 24;
AprilTagPoseData match = poller.tryGetTagId(targetId);

if (match != null && match.t6t_rs != null) {
    telemetry.addData("Tag", "ID %d fam %s", match.fID, match.fam);
    telemetry.addData("Target pose in robot space (x)", match.t6t_rs.x);
    telemetry.addData("Target pose in robot space (y)", match.t6t_rs.y);
    telemetry.addData("Target pose in robot space (z)", match.t6t_rs.z);
    telemetry.addData("Target pose in robot space (pitch)", match.t6t_rs.pitch);
    telemetry.addData("Target pose in robot space (yaw)", match.t6t_rs.yaw);
    telemetry.addData("Target pose in robot space (roll)", match.t6t_rs.roll);
} else {
    telemetry.addLine("Tag 24 not visible");
}
telemetry.update();
```

## What does "Target pose in robot space" mean?

The `t6t_rs` pose is the AprilTag's position and orientation expressed in the robot's coordinate frame.
Put another way: it answers "Where is the tag relative to the robot right now?"

- **Origin:** the robot's coordinate origin (as defined by Limelight's robot-frame configuration).
- **Axes:** x/y/z are measured along the robot's axes in meters.
- **Rotation:** pitch/yaw/roll are the tag's orientation relative to the robot in degrees.

This is the most convenient pose if you want to navigate *from the robot* to the tag, because it is already
described in the robot's own frame of reference.

## Pose field glossary

These names follow Limelight's naming convention: `t6` = 6‑DOF pose, and the suffixes describe the
transform "thing in frame".

- **t6c_ts (Camera pose in target space):** Where the camera is, expressed in the tag's coordinate frame.
- **t6r_ts (Robot pose in target space):** Where the robot is, expressed in the tag's coordinate frame.
- **t6t_cs (Target pose in camera space):** Where the tag is, expressed in the camera's coordinate frame.
- **t6t_rs (Target pose in robot space):** Where the tag is, expressed in the robot's coordinate frame.
- **t6r_fs (Robot pose in field space):** Where the robot is, expressed in the field's coordinate frame.
- **t6r_fs_orb (Robot pose in field space, Megatag2):** Same as `t6r_fs`, but computed by the Megatag2 pipeline.
