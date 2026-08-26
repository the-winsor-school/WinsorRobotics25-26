# Practice & Example Code

Unwired student practice robots, class-exercise components, and demo/example code, moved here during 2026-27 season prep so it stays out of the active build and off the Driver Station OpMode list, without deleting anyone's work.

- `RobotModel/Robots/Wildbots2025.java`, `AppleRobot.java`, `OneStickTankRobot.java`, `StandardTankRobot.java` — alternate practice robots, none wired into any currently-registered OpMode. `AppleRobot` and `OneStickTankRobot` are incomplete (`getAutonomousRobot()` returns `null`).
- `RobotModel/DriveTrain/Tank/OneStickTank.java`, `StandardTankDrive.java` — the drive trains those robots used.
- `RobotModel/Mechs/Assemblies/CascadeArm.java`, `ExampleIntakeAssembly.java` — example mech assemblies for the robots above.
- `RobotModel/Mechs/Components/Claw.java` — example component used by `CascadeArm`/`ExampleIntakeAssembly`. (Note: `DoublyLimitedMotor`, which `CascadeArm` also used, was **kept** in the active tree — it's the worked example for local component-level safety logic in `doc/FlintLessons/03-component-control-strategies.md`.)
- `RobotModel/Mechs/Components/BallDetectionComponent.java` — working color-blob vision component built as a class exercise; never integrated into any `Robot`/`MechAssembly`.
- `AutonStrategies/ExampleAutonomousStrategies.java` — demo autonomous strategy for `Wildbots2025`.
- `OpModes/TeleOp.java` — template/example TeleOp entry point that ran `Wildbots2025`.
- `OpModes/BillyExampleAuton.java`, `SecondExampleAuton.java` — hand-built in-class autonomous exercises on `BillyRobot`, superseded by the (also archived, see `PastSeasons/2025-2026/`) `ATagL1Strategy`/`ATagL2Strategy` routines.

If any of this gets picked up and finished for a new season, `git mv` it back into `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/...` at its original relative path (see each file's original package declaration) to restore it to the build.
