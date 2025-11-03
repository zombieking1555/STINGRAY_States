package frc.robot.reference.referenceSubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.LimelightThreeBase;

public class TurretLimelight extends LimelightThreeBase{
    private final Distance groundToLimelightHeight = Inches.of(0);
    private final Distance limelightToArmAORDistance = Inches.of(0);
    private final Distance limelightToArmAORHeight = Inches.of(0);
    private final Distance groundToAprilTagHeight = Feet.of(3);
    private final Distance aprilTagToTargetHeight = Feet.of(1);
  
  
    public TurretLimelight(String llName, LEDState defaultLEDState) {
        super(llName, defaultLEDState);
     }
    
    /**
     * Method to retrieve horizontal distance from the arm's axis of rotation to the wall on which the april tag resides.
     * @return The distance to the wall
     */
    public Distance getArmDistanceToATWall(){
      //Distance from the limelight to the april tag wall (flat distance) 
      Distance limelightDistanceToAT = (groundToAprilTagHeight.minus(groundToLimelightHeight)).div(Math.tan(getNetworkTableEntry("tx").getDouble(0)));
  
      return limelightDistanceToAT.plus(limelightToArmAORDistance);
    }
  
    /**
     * Method to retrieve the desired arm angle setpoint so that it will point at the target
     * @return the desired setpoint
     */
    public Angle getArmToTargetAngle(){
      Distance armDistToATWall = getArmDistanceToATWall();
      double theta = Math.atan((groundToAprilTagHeight.plus(aprilTagToTargetHeight)).minus(groundToLimelightHeight.plus(limelightToArmAORHeight)).div(armDistToATWall).magnitude());
      return Radians.of(theta);
    }
  
    /**
     * Method to retrieve the error from the current position of the turret to its' desired position
     * @return The error between current and desired turret position (should be added to current position for new setpoint)
     */
    public Angle getTurretRotationError(){
      return Degrees.of(getNetworkTableEntry("ty").getDouble(0));
    }
     
  }