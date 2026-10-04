package frc.robot.subsystems.intake.rollers;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private RollersIO leftRollersIO;
    private RollersIO rightRollersIO;
  private RollersIOInputsAutoLogged leftInputs = new RollersIOInputsAutoLogged();
  private RollersIOInputsAutoLogged rightInputs = new RollersIOInputsAutoLogged();

  public enum RollersState {
    IDLE,
    DEPLOY,
    INTAKE,
    EJECT,
    SMOOSH,
    DISABLED
  }

  public RollersState state = RollersState.DISABLED;

  public Rollers(RollersIO leftRollersIO, RollersIO rightRollersIO) {
    this.leftRollersIO = leftRollersIO;
    this.rightRollersIO = rightRollersIO;
  }

  public void inputsPeriodic() {
    leftRollersIO.updateInputs(leftInputs);
    rightRollersIO.updateInputs(rightInputs);
    Logger.processInputs("Intake/Rollers/Left", leftInputs);
    Logger.processInputs("Intake/Rollers/Right", rightInputs);
  }

  public void setState(RollersState state) {
    this.state = state;
    Logger.recordOutput("Intake/Rollers/state", state);
    }

    // Called at end of command processing in intake
    public void outputsPeriodic() {
      Logger.recordOutput("Intake/Rollers/state", state);
      leftRollersIO.setVoltage(calculateSideVoltage(state));
      rightRollersIO.setVoltage(calculateSideVoltage(state));
    }

    private double calculateSideVoltage(RollersState state) {
        return switch (state) {
          case INTAKE -> (Constants.rollerLeftMode == Constants.SubsystemMode.DISABLED || Constants.rollerRightMode == Constants.SubsystemMode.DISABLED) 
            ? 12.0 : Constants.Rollers.voltageIntake;
          case DEPLOY -> Constants.Rollers.voltageDeploy;
          case EJECT -> Constants.Rollers.voltageEject;
          case SMOOSH -> Constants.Rollers.voltageSmoosh;
          case IDLE, DISABLED -> 0.0;
        };
      }
    
}
