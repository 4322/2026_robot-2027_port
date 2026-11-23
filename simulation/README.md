# Simulator scenarios

Scripted regression tests for the robot simulation. The simulator (`frc.robot.sim.Simulator`)
plays these files through a virtual Xbox controller and the simulated driver station, checks
`expect` events, and writes `results/latest.json`. View robot output in AdvantageScope.

## Running

| What | How |
|---|---|
| Editor + runner GUI | `./gradlew simEditor` |
| Run from the command line | `./gradlew run` (add `-PsimHeadless` for no sim GUI window) |
| What runs | `run.json` (written by the GUI); if missing, `suites/default.json` |
| Exit code | 0 = every check passed, 1 = a check failed or a file was invalid |

If Gradle fails with `Unsupported class file major version`, set `JAVA_HOME` to the WPILib JDK
(e.g. `C:\Users\Public\wpilib\2027_alpha7\jdk`).

## Files

```
field-locations.json   field size, named poses, shapes drawn in the editor, preview speed
scenarios/<id>.json    one scenario per file (id = file name)
suites/<id>.json       ordered list of scenario × alliance runs
run.json               (gitignored) what the next launch runs
results/latest.json    (gitignored) pass/fail report
```

## Scenario format

```json
{
  "name": "Intake Test",
  "description": "optional",
  "gameData": "",            // "", "B" or "R" at the start of the test
  "debugBindings": false,    // log button edges to Sim/Debug
  "auto":   { "routine": "R_CODEORANGE", "duration": 20, "events": [] },
  "teleop": { "startMatchTime": 140, "events": [ ... ] }
}
```

Either segment may be left out. Each event runs `wait` seconds after the previous one. A segment
ends at the later of its last event and its `duration`. Match time counts down from
`startMatchTime` (auto: its duration), so hub-shift logic behaves like a real match.

| `type` | Fields | Notes |
|---|---|---|
| `setPose` | `location` **or** `x`, `y`, `deg`; `frame` | Teleports the robot |
| `drive` | `x` (forward), `y` (left), -1..1 | Left stick, driver-relative, persists |
| `turn` | `rate` -1..1, + = CCW | Right stick X, persists |
| `stopJoysticks` | | Centres both sticks |
| `button` | `button` (A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, BACK, START, LEFT_STICK, RIGHT_STICK), `action` (press, hold, release) | press = one robot loop |
| `pov` | `direction` (CENTER, UP, UP_RIGHT, ...), `action` | |
| `trigger` | `side` (LEFT, RIGHT), `value` 0..1 | Persists |
| `gameData` | `value` ("", "B", "R") | |
| `expect` | `probe` + comparison, optional `within` | Pass/fail check, see below |
| `end` | | Optional; must be last |

Every event may have a `label` (shown in AdvantageScope as `Sim/EventName`).

**Alliances.** Poses are written in BLUE field coordinates. With the default
`"frame": "alliance"` they are mirrored through the field centre when the test runs as RED, so a
scenario means the same thing on both alliances. Use `"frame": "absolute"` for a fixed spot.
Stick values are driver-relative, and the robot code already flips them for RED.

**Expects.** Probes are listed in `frc.robot.sim.scenario.ProbeCatalog`; the editor offers them
in a dropdown.

```json
{ "wait": 0, "type": "expect", "probe": "pose", "location": "OwnLeftTrenchApproach", "tolerance": 0.3, "within": 0.5 }
{ "wait": 0, "type": "expect", "probe": "intake.state", "equals": "EJECT", "within": 0.5 }
{ "wait": 0, "type": "expect", "probe": "drive.speedMps", "min": 2, "within": 1.5 }
```

Without `within` the check runs once. With it, the check passes as soon as it's true at any point
in the next `within` seconds. Right after a `setPose`, give pose checks a short `within`: the
simulated vision can pull the pose back for a moment.

To add a probe, register it in `frc.robot.sim.SimProbes.forRobot()` and add it to `ProbeCatalog`.

## Suite format

```json
{ "name": "Trenches", "runs": [
  { "scenario": "trenches", "alliances": ["BLUE", "RED"] },
  { "autos": "ALL", "alliances": ["BLUE"], "duration": 20 }
] }
```

`"autos": "ALL"` runs every auto routine except the `DRIVE_*` characterization ones.

## Driving manually with a gamepad

1. Plug the gamepad in **before** starting (the sim GUI lists controllers it sees at start-up).
2. Editor Run tab: choose **Manual gamepad**, press Run. The sim GUI window opens.
3. Sim GUI: drag the controller from **System Joysticks** onto slot **1** (not 0) in **Joysticks**,
   and tick **Map gamepad**.
4. In **Robot State**, select **Teleoperated**, then click **Enable** (below the mode list and
   OpMode dropdown). In WPILib 2027 choosing the mode does not enable the robot, and a disabled
   robot ignores all controller input. The DS window shows `Robot Enabled: Yes` once it worked.

Why slot 1: with "Map gamepad" the 2027 sim GUI produces the new 2027 `Gamepad` layout, but the
robot code reads the old NI DS Xbox layout (`NiDsXboxController`); only the left stick and A/B/X/Y
line up. The simulator reads slot 1 every loop, converts it (`frc.robot.sim.GamepadLayout`) and
writes it to Joystick 0, which is what the robot uses. A controller placed directly on slot 0 gets
the wrong layout (left trigger turns the robot, bumpers do nothing).

**AdvantageScope:** in the Joysticks tab, view **Joystick 1**. That's your real controller in the
2027 layout, which AdvantageScope's Xbox layout draws correctly. Joystick 0 holds the converted
copy the robot reads, in the old layout, so AdvantageScope draws everything except the left stick
and A/B/X/Y in the wrong place.

The console shows the state, e.g. `[Sim] Controller: "XInput Controller #1" on Joystick 1 -> robot
Joystick 0 as an Xbox controller` (also logged as `Sim/Manual/Controller`). The sim GUI saves the
assignment in `simgui-ds.json`, so it's a one-time step per controller.

## Changing the editor

The editor is JavaFX. Each tab's layout is an FXML file and all styling is one CSS file, in
`src/simgui/resources/frc/robot/simgui/`:

| File | What it is |
|---|---|
| `main.fxml` | Window frame: scenario list, tabs, Save all |
| `scenario.fxml`, `locations.fxml`, `suites.fxml`, `run.fxml` | One per tab |
| `editor.css` | Theme. The colour palette is the variables at the top of `.root` |

The FXML files open in [Scene Builder](https://gluonhq.com/products/scene-builder/) for
drag-and-drop layout changes. Each one names its controller (`fx:controller`), and the matching
`*Controller.java` in `src/simgui/java/frc/robot/simgui/` holds the behaviour. Two parts are drawn
in code instead: the field view (`FieldCanvas.java`) and the per-event form (`EventEditor.java`),
because their contents change with the data.

## Known limitations

- Subsystem state (intake/shooter state machines) is not reset between tests. Only commands,
  controls, pose and game data are.
- The editor's path preview is a dead-reckoning estimate. It stops at walls but ignores bumps,
  collisions and auto paths.
