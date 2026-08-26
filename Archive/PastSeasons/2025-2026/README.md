# 2025-2026 Season: Game-Specific Autonomous Code

Moved here during 2026-27 season prep because none of it can carry over to a new game, regardless of code quality:

- `AutonStrategies/ATagL1Strategy.java`, `ATagL2Strategy.java` — AprilTag-tracking-and-shoot autonomous routines built for last season's field/tag layout.
- `OpModes/L1AutonBLUE.java`, `L1AutonRED.java`, `L2AutonBLUE.java`, `L2AutonRED.java` — the alliance-specific entry points for the strategies above, each hardcoding a specific AprilTag ID for its alliance/level.

The generic pattern they demonstrate — a `StateMachine`/`IState`-driven autonomous strategy built on `BillyRobot`'s autonomous verb API — is still worth reading as an example; only the game-specific values (tag IDs, alliance split, L1/L2 naming) are stale.
