package frc.robot.subsystems.intake.rollers;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private RollersIO rollersIO;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public enum RollersState {
    IDLE,
    DEPLOY,
    INTAKE,
    EJECT,
    SMOOSH,
    DISABLED
  }

  public RollersState leftState = RollersState.DISABLED;
  public RollersState rightState = RollersState.DISABLED;


  public Rollers(RollersIO rollersIO) {
    this.rollersIO = rollersIO;
  }

  public void inputsPeriodic() {
    rollersIO.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);
  }

  public void setState(RollersState leftState, RollersState righState) {
    this.leftState = leftState;
    // this.rightState = rightState; not doing anything so i'll put it in a comment
    Logger.recordOutput("Intake/Rollers/LeftState", leftState);
    Logger.recordOutput("Intake/Rollers/RightState", rightState);
    }

  // Called at end of command processing in intake
  public void outputsPeriodic() {
    // Nothing here currently
    // Moved below for organizing since its doing nothing
  }

   /* code here previously before, above is a wip

   Logger.recordOutput("Intake/Rollers/State", state);
    switch (state) {
      case DISABLED -> {
        rollersIO.stopMotor();
      }
      case DEPLOY -> {
        rollersIO.setVoltage(Constants.Rollers.voltageDeploy);
      }
      case IDLE -> {
        rollersIO.stopMotor();
      }
      case INTAKE -> {
        rollersIO.setVoltage(Constants.Rollers.voltageIntake);
      }
      case EJECT -> {
        rollersIO.setVoltage(Constants.Rollers.voltageEject);
      }
      case SMOOSH -> {
        rollersIO.setVoltage(Constants.Rollers.voltageSmoosh);
      }
    } */
}
