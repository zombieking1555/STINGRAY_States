package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModuleStatusUtil {
    public static enum ModuleLocation {
        FL,
        FR,
        BL,
        BR;
    }
    public synchronized static void addSwerveModule(ModuleLocation loc, int driveMotorID, int steerMotorID, int encoderID) {
        TalonFX driveMotor = new TalonFX(driveMotorID);
        TalonFX steerMotor = new TalonFX(steerMotorID);
        CANcoder encoder = new CANcoder(encoderID);

        SubsystemStatusManager.addSubsystem("SwerveModule-" + loc.name(), driveMotor, steerMotor, encoder);
        DeviceTempReporter.addDevices(driveMotor, steerMotor);
    }
}
