/**
 * ClockworkLib: reusable motor IO templates for AdvantageKit logging.
 *
 * <p>Provides {@link GenericMotorIO}, {@link TalonFXIO}, and sim implementations
 * {@link GenericRollerIOSim} / {@link GenericPositionIOSim}. Subsystems implement
 * their own IO interface (often extending {@link GenericMotorIO}) and delegate
 * telemetry to a {@link TalonFXIO} handle to avoid duplicated status-signal code.
 */
package frc.robot.util.ClockworkLib;
