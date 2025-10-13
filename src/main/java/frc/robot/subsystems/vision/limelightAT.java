package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightPoseEstimate;

/**
 * Limelight object to represent an aprilTag-measuring limelight.
 */
public class limelightAT {
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

    public limelightAT(String llName, LEDState defaultLEDState) {
        this.llName = llName;
        setLEDState(defaultLEDState);
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
     * @return The yaw of the robot using Metatag1
     */
    public double getMetatagYaw() {
        return getNetworkTableEntry("botpose_wpiblue").getDoubleArray(new double[6])[5];
    }

    /**
     * Method to retrieve the robot pose from this limelight.
     * 
     * @param isMetatag2
     * @return The Limelight Pose Estimate
     */
    public LimelightPoseEstimate getRobotPose(boolean isMetatag2) {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate(llName, "botpose_orb_wpiblue",
                isMetatag2);
        double time = pose.timestampSeconds;
        return new LimelightPoseEstimate(pose, time);
    }

    /**
     * Determines if the given pose is acceptable based on the drivetrain state.
     * 
     * @param pose    The given LimelightPoseEstimate
     * @param dtState The drivetrain state
     * @return Whether the pose is acceptable.
     */
    public boolean isPoseOk(LimelightPoseEstimate pose, SwerveDriveState dtState) {
        return pose != null
                && pose.getPose() != null
                && pose.getPoseEstimate().tagCount > 0
                && Math.abs(Units.radiansToRotations(dtState.Speeds.omegaRadiansPerSecond)) < 2;
    }

    /**
     * Method to set the orientation of the limelight relative to the field
     * 
     * @param dtState State of the robot drivetrain
     * @apiNote This method MUST be called periodically.
     */
    public void setRobotOrientation(SwerveDriveState dtState) {
        LimelightHelpers.SetRobotOrientation(llName, dtState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    /**
     * Sets the state of the limelight LEDs.
     * 
     * @param state The state to set the LEDs to
     */
    public void setLEDState(LEDState state) {
        LimelightHelpers.setLimelightNTDouble(llName, "ledMode", state.getLEDMode());
    }
}
