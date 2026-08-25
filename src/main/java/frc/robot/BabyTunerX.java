package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.wpilib.driverstation.DriverStationErrors;

public class BabyTunerX {

    private static boolean init;

    private static String subsystemName;

    private static Timer initTimer = new Timer();

    private static boolean pidMode = true;

    private static TalonFXConfiguration config = new TalonFXConfiguration();

    private static StatusCode status;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("kP");

    private static final LoggedTunableNumber kI = new LoggedTunableNumber("kI");

    private static final LoggedTunableNumber iSat = new LoggedTunableNumber("iSat");

    private static final LoggedTunableNumber iZone = new LoggedTunableNumber("iZone");

    private static final LoggedTunableNumber kD = new LoggedTunableNumber("kD");

    private static final LoggedTunableNumber kG = new LoggedTunableNumber("kG");

    private static final LoggedTunableNumber kV = new LoggedTunableNumber("kV");

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("kS");

    private static final LoggedTunableNumber acc = new LoggedTunableNumber("max acc");

    private static final LoggedTunableNumber dec = new LoggedTunableNumber("max dec");

    private static final LoggedTunableNumber vel = new LoggedTunableNumber("max vel");

    private static final LoggedTunableNumber setpoint = new LoggedTunableNumber("PID setpoint");

    private static final LoggedTunableNumber voltage1 = new LoggedTunableNumber("Set voltage motor 1");

    private static final LoggedTunableNumber voltage2 = new LoggedTunableNumber("Set voltage motor 2");

    private static String subsystemKey = "/Tuning/Subsystem";

    private static String currentValueKey = "/Tuning/Current Value";

    private static String feedbackErrorKey = "/Tuning/Error";

    private static String unitsKey = "/Tuning/Units";

    private static NetworkTableEntry subsystemEntry = NetworkTableInstance.getDefault().getEntry(subsystemKey);

    private static NetworkTableEntry currentValueEntry = NetworkTableInstance.getDefault().getEntry(currentValueKey);

    private static NetworkTableEntry feedbackErrorEntry = NetworkTableInstance.getDefault().getEntry(feedbackErrorKey);

    private static NetworkTableEntry unitsEntry = NetworkTableInstance.getDefault().getEntry(unitsKey);

    public static Double run(int motorIdx, TalonFX talonFX, String subsystemName, double currentValue, String unitString) {
        Double newPos = null;
        boolean configChanged = false;
        initTimer.start();
        // wait for settings to be received from the nitrate
        if (!initTimer.hasElapsed(1.0)) {
            return newPos;
        }
        if (!init) {
            BabyTunerX.subsystemName = subsystemName;
            if (Constants.currentMode == Mode.REAL) {
                status = talonFX.getConfigurator().refresh(config);
            } else {
                // for testing GUI in sim mode
                config.Slot0.kP = 1;
                config.Slot0.kI = 2;
                config.Slot0.kD = 3;
                config.Slot0.kG = 4;
                config.Slot0.kS = 5;
                config.Slot0.kV = 6;
                config.MotionMagic.MotionMagicAcceleration = 7;
                config.MotionMagic.MotionMagicCruiseVelocity = 8;
            }
            setDefault(motorIdx, kP, config.Slot0.kP);
            setDefault(motorIdx, kI, config.Slot0.kI);
            setDefault(motorIdx, iSat, 0);
            setDefault(motorIdx, iZone, 0);
            setDefault(motorIdx, kD, config.Slot0.kD);
            setDefault(motorIdx, kG, config.Slot0.kG);
            setDefault(motorIdx, kV, config.Slot0.kV);
            setDefault(motorIdx, kS, config.Slot0.kS);
            setDefault(motorIdx, acc, config.MotionMagic.MotionMagicAcceleration);
            setDefault(motorIdx, dec, 0);
            setDefault(motorIdx, vel, config.MotionMagic.MotionMagicCruiseVelocity);
            setDefault(motorIdx, setpoint, currentValue);
            setDefault(motorIdx, voltage1, 0);
            setDefault(motorIdx, voltage2, 0);
            subsystemEntry.setString(subsystemName);
            currentValueEntry.setString("");
            feedbackErrorEntry.setString("");
            unitsEntry.setString(unitString);
            init = true;
            // can't continue because all settings will still show as changed in the current time tick
            return newPos;
        }
        if (BabyTunerX.subsystemName != subsystemName) {
            DriverStationErrors.reportError("Tuning multiple subsystems is not supported", false);
            System.exit(1);
        }
        if (motorIdx == 0) {
            if (pidMode) {
                currentValueEntry.setString(String.format("%.4f", currentValue));
                feedbackErrorEntry.setString(String.format("%.4f", setpoint.get() - currentValue));
            } else {
                currentValueEntry.setString("");
                feedbackErrorEntry.setString("");
            }
        }
        if (kP.hasChanged(motorIdx)) {
            config.Slot0.kP = kP.get();
            configChanged = true;
        }
        if (kI.hasChanged(motorIdx)) {
            config.Slot0.kI = kI.get();
            configChanged = true;
        }
        if (kD.hasChanged(motorIdx)) {
            config.Slot0.kD = kD.get();
            configChanged = true;
        }
        if (kG.hasChanged(motorIdx)) {
            config.Slot0.kG = kG.get();
            configChanged = true;
        }
        if (kV.hasChanged(motorIdx)) {
            config.Slot0.kV = kV.get();
            configChanged = true;
        }
        if (kS.hasChanged(motorIdx)) {
            config.Slot0.kS = kS.get();
            configChanged = true;
        }
        if (acc.hasChanged(motorIdx)) {
            config.MotionMagic.MotionMagicAcceleration = acc.get();
            configChanged = true;
        }
        if (vel.hasChanged(motorIdx)) {
            config.MotionMagic.MotionMagicCruiseVelocity = vel.get();
            configChanged = true;
        }
        if (setpoint.hasChanged(motorIdx)) {
            newPos = setpoint.get();
            pidMode = true;
        }
        if (motorIdx == 0 && voltage1.hasChanged(motorIdx)) {
            if (talonFX != null) {
                talonFX.setVoltage(voltage1.get());
            }
            pidMode = false;
        }
        if (motorIdx == 1 && voltage2.hasChanged(motorIdx)) {
            if (talonFX != null) {
                talonFX.setVoltage(voltage2.get());
            }
            pidMode = false;
        }
        if (configChanged && (Constants.currentMode == Mode.REAL)) {
            status = talonFX.getConfigurator().apply(config);
            if (status != StatusCode.OK) {
                DriverStationErrors.reportError("Talon " + talonFX.getDeviceID() + " error (Right Elevator): " + status.getDescription(), false);
            }
        }
        return newPos;
    }

    private static void setDefault(int motorIdx, LoggedTunableNumber tunable, double val) {
        tunable.initDefault(val);
        // throw away initial settings change
        if (!tunable.hasChanged(motorIdx)) {
            DriverStationErrors.reportError("This should never happen", true);
            System.exit(0);
        }
    }
}
