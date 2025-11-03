package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SubsystemStatusManager;

public class LimelightThreeBase {
    public enum LEDState {
        OFF(1),
        BLINK(2),
        ON(3);

        private int mode;

        private LEDState(int mode) {
            this.mode = mode;
        }

        public int getLEDMode() {
            return mode;
        }
    }

    private String llName;

    public LimelightThreeBase(String llName, LEDState defaultLEDState) {
         this.llName = llName;
        setLEDState(defaultLEDState);
        SubsystemStatusManager.addSubsystem(llName, ()-> NetworkTableInstance.getDefault().getTable(llName).getTopic("tv").exists());
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
     * Sets the state of the limelight LEDs.
     * 
     * @param state The state to set the LEDs to
     */
    public void setLEDState(LEDState state) {
        LimelightHelpers.setLimelightNTDouble(llName, "ledMode", state.getLEDMode());
    }

    /**
     * Blink the LEDs of the Limelight 3
     * @param seconds Seconds of blink time
     * @return The LED blink command
     */
    public Command blinkLEDs(double seconds){
        return new StartEndCommand(()-> setLEDState(LEDState.BLINK), ()-> setLEDState(LEDState.OFF)).withTimeout(seconds);
    }
}
