package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    //left seperate motor boolean
    public boolean leftConnected = false;
    public double leftStatorCurrentAmps = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTempCelcius = 0.0;
    public double leftRotationsPerSec = 0.0;
    public double leftVolts = 0.0;

    //right seperate motor boolean
    public boolean rightConnected = false;
    public double rightStatorCurrentAmps = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTempCelcius = 0.0;
    public double rightRotationsPerSec = 0.0;
    public double rightVolts = 0.0;
  }
  
  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void stop() {}

  public default TalonFX getLeftTalonFX() { return null; }
  public default TalonFX getRightTalonFX() { return null; }
}
