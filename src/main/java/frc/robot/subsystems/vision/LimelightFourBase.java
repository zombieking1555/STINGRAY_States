package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DeviceTempReporter;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SubsystemStatusManager;

public class LimelightFourBase extends SubsystemBase {
    /**
     * States corresponding to the IMU mode of the Limelight
     */
    public enum IMUState {
        EXTERNAL(0),
        EXTERNAL_SEED_INTERNAL(2),
        INTERNAL(2),
        INTERNAL_MT1_CONVERGE(3),
        INTERNAL_EXTERNAL_CONVERGE(4);

        private int mode;

        private IMUState(int mode) {
            this.mode = mode;
        }

        public int getIMUMode() {
            return mode;
        }
    }

    private String llName;
    private IMUState currentState;
    private ShuffleboardTab llTab = Shuffleboard.getTab("limelight");

    public LimelightFourBase(String llName, IMUState defaultIMUState) {
         this.llName = llName;
        setIMUState(defaultIMUState);
        currentState = defaultIMUState;

        llTab.addString(llName + " State", ()-> getIMUState().name());
        SubsystemStatusManager.addSubsystem(llName, ()-> NetworkTableInstance.getDefault().getTable(llName).getTopic("tv").exists());
        DeviceTempReporter.addDevice(llName, ()->getDeviceTemp());
    }

    /**
     * Method used to retrieve a networktable value from this limelight.
     * 
     * @param entryName The name of the entry to rethrn
     * @return The entry to return
     */
    public NetworkTableEntry getNetworkTableEntry(String entryName) {
        return LimelightHelpers.getLimelightNTTableEntry(llName, entryName);
    }

    /**
     * Method to retrieve the temperature of the limelight
     * @return The device temperature
     * TODO: Find the actual unit for this measurement
     */
    public Temperature getDeviceTemp(){
        double temp = NetworkTableInstance.getDefault().getTable(llName).getEntry("hw").getDoubleArray(new double[4])[3];
        return Celsius.of(temp); //Celsius is a guess. i dont know what unit it actually is.
    }
    
    /**
     * Sets the state of the limelight LEDs.
     * 
     * @param state The state to set the LEDs to
     */
    public void setIMUState(IMUState state) {
        currentState = state;
        LimelightHelpers.setLimelightNTDouble(llName, "imumode_set", state.getIMUMode());
    }

    public IMUState getIMUState(){
        return currentState;
    }
}
