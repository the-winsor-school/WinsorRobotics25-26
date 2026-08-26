# Archive

Code moved out of `TeamCode/src/main/java/...` during 2025-26 -> 2026-27 season prep (branch `2027-season-prep`). Everything here is **excluded from the Gradle build** (it's outside any module's source set) and will not appear on the Driver Station.

Nothing was deleted — every file kept its git history via `git mv`, so `git log --follow -- Archive/.../SomeFile.java` still shows its full history from when it lived under `TeamCode/`.

- `PastSeasons/2025-2026/` — this season's game-specific autonomous code (hardcoded AprilTag IDs, alliance-specific OpModes). Will not carry over to a new game regardless of code quality.
- `PracticeAndExamples/` — unwired student practice robots, class-exercise components, and demo/example code from throughout the season. Preserved so nobody's work is lost, kept out of the active build.
- `Deprecated/` — dead stubs, superseded prototypes, and one unmodified vendor SDK sample file that had been copied into `TeamCode/`.

See each subfolder's README for details on what's in it and why it was moved.

The active `RobotModel` framework (`Robot`/`DriveTrain`/`MechAssembly`/`MechComponent` and friends) and `BillyRobot`/`BillyMA` were kept in the live tree as the team's ongoing reference example — they're what `doc/AbstractRobotObjectModel.md`, `doc/NewRobotDesignWorkflow.md`, and `doc/FlintLessons/` teach from.
