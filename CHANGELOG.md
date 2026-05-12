# Changelog

## Telemetry Injection Refactor

**Branch:** `telemetry-refactor`  
**Commit:** `ffb3976`

### Summary

Replaced the loop-time `updateTelemetry(Telemetry)` parameter pattern with
constructor/initialization-time injection. The central contract is now:

- `Telemetry` is injected **once** via `initializeTelemetry(Telemetry)` (called from `Robot.initializeSubsystems()`)
- `Robot.updateTelemetry()` is the **single flush point** — the only place that calls `telemetry.update()`
- Autonomous inner classes expose `reportStatus(String)` / `reportData(String, Object)` and never call `telemetry.update()` directly

---

### Bug Fixes

| # | Description | Fixed in |
|---|-------------|----------|
| **#2** | `BillyRapidFire` and `LimelightAutoTarget` called `telemetry.update()` on every state transition, causing extra flushes mid-loop | `BillyRapidFire.java`, `LimelightAutoTarget.java` |
| **#4** | `PusherServo.update()` ignored its `telemetryStrategy` entirely | `PusherServo.java` |
| **#5** | `BillyMA.updateTelemetry()` never called `intake.update()` | `BillyMA.java` |
| **#6** | `LimelightAutoTarget` called `telemetry.update()` from within state lambdas | `LimelightAutoTarget.java` |
| **#7** | `BillyMA.giveInstructions()` called `telemetry.update()` mid-cycle on rapid fire start | `BillyMA.java` |

---

### Files Changed

#### Base Abstract Classes

**`MechComponent.java`**
- Added `protected Telemetry telemetry` field
- Added concrete `initializeTelemetry(Telemetry)` method
- `AutonomousComponentBehaviors` inner class: new `(Telemetry)` constructor, `reportStatus(String)`, `reportData(String, Object)`
- `update(Telemetry)` → `abstract void update()` (no parameter)

**`DriveTrain.java`**
- Added `protected Telemetry telemetry` field
- Added `public abstract void initializeTelemetry(Telemetry)`
- `updateTelemetry(Telemetry)` → `public abstract void updateTelemetry()` (no parameter)
- `AutonomousDriving` inner class: new `(Telemetry)` constructor, `reportStatus(String)`, `reportData(String, Object)`

**`MechAssembly.java`**
- Added `protected Telemetry telemetry` field
- Added `public abstract void initializeTelemetry(Telemetry)`
- `updateTelemetry(Telemetry)` → `public abstract void updateTelemetry()` (no parameter)
- `AutonomousMechBehaviors` inner class: new `(Telemetry)` constructor, `reportStatus(String)`, `reportData(String, Object)`

**`Robot.java`**
- Added `protected Telemetry telemetry` field
- Added `protected Robot(Telemetry)` constructor
- Added `protected void initializeSubsystems()` — propagates telemetry to driveTrain and mechAssembly
- `updateTelemetry(Telemetry)` → `public void updateTelemetry()` — now the single flush point (calls `telemetry.update()`)
- `AutonomousRobot` inner class: new `(DriveTrain.AutonomousDriving, MechAssembly.AutonomousMechBehaviors, Telemetry)` constructor; exposes `reportStatus`, `reportData`, `clearTelemetry`
- `update(Gamepad, Gamepad)` now null-guards driveTrain and mechAssembly

---

#### Drive Trains

**`MecanumDrive.java`**
- `AutonomousMecanumDrive` gains `(Telemetry)` constructor calling `super(telemetry)`
- `auton` field changed from eagerly initialized `final` to lazily initialized in `initializeTelemetry`
- Added `initializeTelemetry(Telemetry)` — sets field and creates `auton`
- `updateTelemetry(Telemetry)` → `updateTelemetry()` — writes IMU yaw and motor powers via `this.telemetry`
- `turnToAngle()` and `drive()` in `AutonomousMecanumDrive` use `reportData/reportStatus`

**`OneStickTank.java`**
- `LeroyState` constructor: `(double chargeTime)` → `(double chargeTime, Telemetry telemetry)`, calls `super(telemetry)`
- `activate()` and `isCompleted()` use `reportStatus/reportData`
- `leroyState` field changed from eagerly initialized `final` to lazily initialized in `initializeTelemetry`
- Added `initializeTelemetry(Telemetry)` — creates `leroyState`
- `updateTelemetry(Telemetry)` → `updateTelemetry()` — writes left/right power and Leroy active state

**`StandardTankDrive.java`** *(cascading fix)*
- `AutonomousTankDrive` gains `(Telemetry)` constructor; movement methods use `reportStatus`
- `auton` field lazily initialized in `initializeTelemetry`
- Added `initializeTelemetry(Telemetry)`
- `updateTelemetry(Telemetry)` → `updateTelemetry()` — writes left/right power

---

#### Mech Components

**`SpinnyIntake.java`**
- `AutonomousIntakeBehaviors` gains `(Telemetry)` constructor; `startIntake/stopIntake/reverseIntake` use `reportStatus`
- `auton` field lazily initialized in `initializeTelemetry`
- Added `initializeTelemetry(Telemetry)` calling `super` then creating `auton`
- `update(Telemetry)` → `update()` — writes intake power and position

**`DoubleShooter.java`**
- `AutonomousShooterBehavior` gains `(Telemetry)` constructor; `StartShoot/StopShoot/setPower` use `reportStatus/reportData`
- `getAutonomousBehaviors()` now returns a stable `auton` field instead of allocating `new AutonomousShooterBehavior()` on every call
- Added `initializeTelemetry(Telemetry)` creating `auton`
- `update(Telemetry)` → `update()` — calls `telemetryStrategy` if present, else writes direct

**`PusherServo.java`** *(bug #4 fix)*
- `AutonomousBallPusherBehaviors` gains `(Telemetry)` constructor; `pushBalls/setPosition/retractPusher` use `reportStatus/reportData`
- `auton` field lazily initialized in `initializeTelemetry`
- `update(Telemetry)` → `update()` — now calls `telemetryStrategy.update(servoR, telemetry)` when strategy is present (was previously not called)

**`Turret.java`**
- `AutonomousTurretBehaviors` gains `(Telemetry)` constructor; `setPower/turnCCW/turnCW/stop` use `reportData/reportStatus`
- `auton` field lazily initialized in `initializeTelemetry`
- `update(Telemetry)` → `update()` — calls strategy or writes direct

**`Claw.java`**
- `AutonomousClawBehaviors` gains `(Telemetry)` constructor; `open/close` use `reportStatus`; added `stop()` method
- `auton` field lazily initialized in `initializeTelemetry`
- `update(Telemetry)` → `update()` — calls strategy or writes direct

**`DoublyLimitedMotor.java`**
- `AutonomousDLMBehaviors` gains `(Telemetry)` constructor; `goForward/stop/goBackward` use `reportStatus`
- `auton` field lazily initialized in `initializeTelemetry`
- `update(Telemetry)` → `update()` — calls strategy or writes motor power + sensor states

**`BallDetectionComponent.java`** *(cascading fix)*
- `AutonomousBallDetection` gains `(Telemetry)` constructor
- `auton` field lazily initialized in `initializeTelemetry`
- `update(Telemetry)` → `update()` — same body, uses `this.telemetry`

---

#### Mech Assemblies

**`BillyMA.java`** *(bug #5 and #7 fixes)*
- Constructor signature: `BillyMA(HardwareMap, Telemetry)` → `BillyMA(HardwareMap)` — telemetry arrives via `initializeTelemetry`
- `initializeTelemetry(Telemetry)` propagates to all 4 components, then creates `auton` and `BRF`
- `BillyRapidFire` construction: `new BillyRapidFire(auton, 3, tel)` → `new BillyRapidFire(auton, 3)`
- `AutonomousBillyMA` constructor gains `Telemetry` parameter, calls `super(telemetry)`
- `giveInstructions()` removes `telemetry.update()` call on rapid-fire start (bug #7)
- `updateTelemetry()` now calls `intake.update()` in addition to the other three components (bug #5)

**`CascadeArm.java`**
- `AutonomousCascadeArm` constructor gains `Telemetry` parameter, calls `super(telemetry)`
- `auton` field lazily initialized in `initializeTelemetry`
- `initializeTelemetry(Telemetry)` propagates to cascade, drawbridge, and claw
- `updateTelemetry(Telemetry)` → `updateTelemetry()` — writes section header then calls component `update()`

**`ExampleIntakeAssembly.java`** *(cascading fix)*
- `AutonomousIntakeAssembly` constructor gains `Telemetry` parameter, calls `super(telemetry)`
- `auton` field lazily initialized in `initializeTelemetry`
- `initializeTelemetry(Telemetry)` propagates to intake and claw
- `updateTelemetry(Telemetry)` → `updateTelemetry()` — calls `claw.update()` and `intake.update()`

---

#### Robots

**`BillyRobot.java`**
- Constructor calls `super(telemetry)` first
- `new BillyMA(hardwareMap, telemetry)` → `new BillyMA(hardwareMap)`
- Calls `initializeSubsystems()` after driveTrain and mechAssembly are assigned
- `AutonomousMecanumRobot` constructor: adds `Telemetry` parameter, calls `super(driveTrain, mechAssembly, telemetry)`
- `new LimelightAutoTarget(limelight, turret, telemetry, tagID)` → `new LimelightAutoTarget(limelight, turret, tagID)`

**`Wildbots2025.java`** *(cascading fix)*
- Constructor: `Wildbots2025(HardwareMap)` → `Wildbots2025(HardwareMap, Telemetry)`, calls `super(telemetry)`
- Calls `initializeSubsystems()` before creating `auton`
- `AutonomousMecanumRobot` constructor gains `Telemetry` parameter

**`OneStickTankRobot.java`** *(cascading fix)*
- Constructor: gains `Telemetry` parameter, calls `super(telemetry)` and `initializeSubsystems()`

**`AppleRobot.java`** *(cascading fix)*
- Constructor: gains `Telemetry` parameter, calls `super(telemetry)` and `initializeSubsystems()`

**`StandardTankRobot.java`** *(cascading fix)*
- Constructor: gains `Telemetry` parameter, calls `super(telemetry)` and `initializeSubsystems()`

---

#### Autonomous Strategies

**`BillyRapidFire.java`** *(bug #2 fix)*
- Constructor: `BillyRapidFire(AutonomousBillyMA, int, Telemetry)` → `BillyRapidFire(AutonomousBillyMA, int)` — no Telemetry parameter
- `startShooter/fire/retract/stopShooter` replace `telemetry.addLine/update()` with `mechAssembly.reportStatus()`

**`LimelightAutoTarget.java`** *(bugs #2 and #6 fix)*
- Constructor: removes `Telemetry` parameter
- `rotateCCW/rotateCW/stopTurret/lookForTag` replace direct `telemetry.addData/update()` calls with `turret.reportData/reportStatus()`

---

#### OpModes

**`TeleOp.java`** *(cascading fix)*
- `new Wildbots2025(hardwareMap)` → `new Wildbots2025(hardwareMap, telemetry)`
- `robot.updateTelemetry(telemetry)` → `robot.updateTelemetry()`
- Removed redundant `telemetry.update()` call in loop (now handled by `Robot.updateTelemetry()`)

**`BillyTeleOpRED.java`** *(cascading fix)*
- `robot.updateTelemetry(telemetry)` → `robot.updateTelemetry()`
- Removed redundant `telemetry.update()` call in loop

**`BillyTeleOpBLUE.java`** *(cascading fix)*
- `robot.updateTelemetry(telemetry)` → `robot.updateTelemetry()`
- Removed redundant `telemetry.update()` call in loop

**`BillyExampleAuton.java`** *(cascading fix)*
- `new LimelightAutoTarget(limelight, turret, telemetry, 20)` → `new LimelightAutoTarget(limelight, turret, 20)`
