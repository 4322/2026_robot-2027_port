# Plan: ClockworkLib Motor IO Templates

## Goal
Create reusable, documented motor IO templates in `frc.robot.util.ClockworkLib`, then refactor every `*IOTalonFX`/`*IO` in the project to use them, minimizing duplicated AdvantageKit logging plumbing.

## Target location
`src/main/java/frc/robot/util/ClockworkLib/`

## Files to create
- `GenericMotorIO.java` — interface contract + `@AutoLog GenericMotorIOInputs` (connected, stator/supply current, temp, velocity RPS, position deg, applied volts) with Javadoc.
- `TalonFXIO.java` — concrete TalonFX implementation owning status signals, debouncer, config apply, bus optimization. Constructor `(int id, CANBus bus, TalonFXConfiguration config, double updateHz)`. Implements `GenericMotorIO`.
- `GenericRollerIOSim.java` — sim for velocity-controlled rollers/indexers/flywheels (implements `GenericMotorIO`; models velocity response to voltage, current, temp).
- `GenericPositionIOSim.java` — sim for position-controlled mechanisms (arm/elevator/hood); models position + velocity + current + temp with gravity/load approximations.
- `package-info.java` — package overview + usage example.

## Refactor tasks (ordered)
1. Move existing `frc.robot.subsystems.genericsubsystems.GenericMotorIO` and `TalonFXIO` into `ClockworkLib`; update imports across `RollersIO`, etc.
2. Convert single-motor IOs (`RollersIOTalonFX`, `DeployerIOTalonFX`, `HoodIOTalonFx`, `TurretIOTalonFx`, `SpindexerIOTalonFx`, `TunnelIOTalonFx`) to instantiate `TalonFXIO` and delegate `updateInputs`/`setVoltage`/`stop`/`getTalonFX`.
3. Convert multi-motor IOs (`FlywheelIOTalonFx`, `ModuleIOTalonFX`) to hold one `TalonFXIO` per motor and delegate telemetry; keep subsystem-specific control (followers, velocity requests, slots) in the IO class.
4. Replace existing `*IOSim` implementations (`RollersIOSim`, `FlywheelIOSim`, `HoodIOSim`, `TurretIOSim`, `SpindexerIOSim`, `TunnelIOSim`, `DeployerIOSim`) with `GenericRollerIOSim`/`GenericPositionIOSim` instances tuned per subsystem.
5. Standardize IO interfaces: simple single-motor ones `extends GenericMotorIO`; multi-motor ones compose `GenericMotorIOInputs` fields (e.g. `leader`, `follower`) inside their `@AutoLog` inputs.
6. Update subsystem classes and `RobotContainer` wiring; remove now-unused status-signal/debouncer imports.
7. Add Javadoc to all public members.

## Risks
- Field-name casing differences (e.g. `inputs.Volts` vs `appliedVolts`) require updating subsystem consumers; verify each `*Inputs` usage.
- AdvantageKit generated `*IOInputsAutoLogged` classes regenerate at build; confirm compile after refactor.
- Sim implementations must return consistent defaults.

## Validation
- `./gradlew compileJava`
- Run existing tests / sim mode; check AdvantageKit log fields unchanged.
