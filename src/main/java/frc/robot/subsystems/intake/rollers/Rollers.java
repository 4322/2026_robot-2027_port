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

  public void setState(RollersState state) {
    this.leftState = Constants.Rollers.enableLeftMotor ? state : RollersState.DISABLED;
    this.rightState = Constants.Rollers.enableRightMotor ? state : RollersState.DISABLED;
    Logger.recordOutput("Intake/Rollers/LeftState", leftState);
    Logger.recordOutput("Intake/Rollers/RightState", rightState);
    }

    // Called at end of command processing in intake
    public void outputsPeriodic() {
      Logger.recordOutput("Intake/Rollers/LeftState", leftState);
      Logger.recordOutput("Intake/Rollers/RightState", rightState);

      rollersIO.setVoltage(calculateSideVoltage(leftState), calculateSideVoltage(rightState));
    }

    private double calculateSideVoltage(RollersState state) {
        return switch (state) {
          case INTAKE -> Constants.Rollers.voltageIntake;
          case DEPLOY -> Constants.Rollers.voltageDeploy;
          case EJECT -> Constants.Rollers.voltageEject;
          case SMOOSH -> Constants.Rollers.voltageSmoosh;
          case IDLE, DISABLED -> 0.0;
        };
      }
    
}
