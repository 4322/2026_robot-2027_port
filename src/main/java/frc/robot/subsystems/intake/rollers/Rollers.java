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
    this.rightState = rightState;
    Logger.recordOutput("Intake/Rollers/LeftState", leftState);
    Logger.recordOutput("Intake/Rollers/RightState", rightState);
    }

    public void setState(RollersState state) {
      setState(state, state);
    }

    // Called at end of command processing in intake
    public void outputsPeriodic() {
      double leftVoltage = calculateSideVoltage(leftState, rightState);
      double rightVoltage = calculateSideVoltage(rightState, leftState);

      rollersIO.setVoltage(leftVoltage, rightVoltage);
    }

    private double calculateSideVoltage(RollersState myState, RollersState otherState) {
      if (myState == RollersState.DISABLED) {
        return  0.0;
      }
      
      if (myState == RollersState.INTAKE) {
        if (otherState == RollersState.INTAKE) {
          if (otherState == RollersState.DISABLED) {
            return 12.0;
          }
          return Constants.Rollers.voltageIntake;
        }

        return switch (myState) { //not sure how to fix myState error
          case DEPLOY -> Constants.Rollers.voltageDeploy;
          case EJECT -> Constants.Rollers.voltageEject;
          case SMOOSH -> Constants.Rollers.voltageSmoosh;
          case IDLE, DISABLED -> 0.0;
        };
      }
    }
  /* otherState = The current state of the opposite side of the intake
     myState = The current state of the motor you're currently calculating
     (I think monkey thinking face)


    code here previously before, above is a wip

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
