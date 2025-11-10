// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TeleopSwerve extends Command {
  private enum driveMode{
    FALCONDRIVE,
    NORMALDRIVE;
  }
  private CommandSwerveDrivetrain dt;
  private Elevator elevator;
  private CommandXboxController controller;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  
  private SwerveRequest.FieldCentric driveNormal;
  
  private SwerveRequest.FieldCentric driveSlow;

  private SwerveRequest.FieldCentric driveSlew;

  private SwerveRequest.FieldCentric falconDrive;

  private GenericEntry swerveCommandEntry;
  private String swerveCommandType = "NORMAL";
  private final ShuffleboardTab tab = Shuffleboard.getTab(getName());
  private final SendableChooser<TeleopSwerve.driveMode> driveModeChooser = new SendableChooser<>();

   // Stores the joystick's angle from the previous loop cycle to calculate its speed
   private Angle previousJoystickAngle = Rotations.of(0); 
   // A timestamp from the last time the angle was updated, for a more accurate velocity calculation
   private Time lastAngleTimestamp = Seconds.of(0);
   private Rotation2d lastTargetDirection = Rotation2d.fromDegrees(0);

  /** Command used to control swerve in teleop. */
  public TeleopSwerve(CommandSwerveDrivetrain dt, Elevator elevator, CommandXboxController controller) {
    this.dt = dt;
    this.elevator = elevator;
    this.controller = controller;

    driveModeChooser.addOption("Falcon Drive", driveMode.FALCONDRIVE);
    driveModeChooser.setDefaultOption("Normal Drive", driveMode.NORMALDRIVE);
    tab.add("Drive Mode Chooser", driveModeChooser);

    driveNormal = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);
    

    driveSlow = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);
    

    driveSlew = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);

    falconDrive = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*0.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*0.07);

    swerveCommandEntry= tab.add("Swerve Command Status", swerveCommandType).getEntry();

    addRequirements(this.dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (driveModeChooser.getSelected()) {
      case FALCONDRIVE:
      
        double rightX = -controller.getRightX();
        double rightY = -controller.getRightY();

        if (Math.hypot(rightX, rightY) > 0.7) { // Deadband check
            // --- PREDICTION LOGIC START ---
            
            // 1. Calculate current angle and normalize to [0, 360)
            Angle currentJoystickAngle = Radians.of(Math.atan2(-rightY, -rightX) + Math.toRadians(90));
            currentJoystickAngle = Degrees.of((currentJoystickAngle.in(Degrees) % 360 + 360) % 360);

            // 2. Calculate joystick's angular velocity
            Time currentTime = Seconds.of(Timer.getFPGATimestamp());
            Time deltaTime = currentTime.minus(lastAngleTimestamp);
            
            // To avoid division by zero on the first run
            if (deltaTime.isEquivalent(Seconds.of(0))) {
                deltaTime = Seconds.of(0.02); // Assume a 50Hz loop rate initially
            }

            // Find the shortest angle difference to handle the 359 -> 0 degree wrap-around
            Angle angleDifference = currentJoystickAngle.minus(previousJoystickAngle);
            if (angleDifference.gt(Degrees.of(180))) angleDifference = angleDifference.minus(Degrees.of(360));
            if (angleDifference.lt(Degrees.of(-180))) angleDifference = angleDifference.plus(Degrees.of(360));

            AngularVelocity joystickAngularVelocity = angleDifference.div(deltaTime); // degrees per second

            // 3. Calculate the "lead" angle based on velocity
            // This is the amount we'll predict ahead. The faster the stick moves, the more we lead.
            // The '0.05' is a tuning factor. Increase it for more prediction, decrease for less.
            Angle leadAngle = joystickAngularVelocity.times(Seconds.of(.05));

            // Clamp the lead angle to a reasonable maximum (e.g., 20 degrees) to prevent instability
            
            leadAngle = Degrees.of(Math.max(-20.0, Math.min(20.0, leadAngle.in(Degrees))));

            // 4. Determine the base snapped angle
            Angle baseSnappedAngle = Degrees.of(Math.round(currentJoystickAngle.in(Degrees) / 30.0) * 30.0);

            // 5. Apply the prediction
            // The final target is the snapped angle plus our predictive lead
            Angle finalTargetAngle = baseSnappedAngle.plus(leadAngle);

            lastTargetDirection = new Rotation2d(finalTargetAngle);
            // --- PREDICTION LOGIC END ---

            // Update state for the next loop
            previousJoystickAngle = currentJoystickAngle;
            lastAngleTimestamp = currentTime;
        }
        
        Rotation2d currentRotation = dt.getState().Pose.getRotation();
        Angle errorRad = lastTargetDirection.minus(currentRotation).getMeasure();
        //Angular velocity of 5 times the error
        AngularVelocity rotationalRateRadPerSec = errorRad.div(Seconds.of(0.2));

        rotationalRateRadPerSec = AngularVelocity.ofBaseUnits(Math.max(-Math.PI, Math.min(Math.PI, rotationalRateRadPerSec.in(RadiansPerSecond))), RadiansPerSecond);

        dt.setControl(falconDrive
        .withVelocityX(-controller.getLeftY() * SwerveConstants.MAX_SPEED)
        .withVelocityY(-controller.getLeftX() * SwerveConstants.MAX_SPEED)
        .withRotationalRate(rotationalRateRadPerSec));

        swerveCommandType = "FALCON";
        swerveCommandEntry.setString(swerveCommandType);

        break;
    
      case NORMALDRIVE:

      switch(elevator.getCurrentState()){
        case L4:
          dt.setControl(driveSlew
          .withVelocityX(xLimiter.calculate(-controller.getLeftY() * SwerveConstants.MAX_SPEED))
          .withVelocityY(yLimiter.calculate(-controller.getLeftX() * SwerveConstants.MAX_SPEED))
          .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));
  
          swerveCommandType = "SLEW";
          swerveCommandEntry.setString(swerveCommandType);
  
          break;
        case BARGE:
          dt.setControl(driveSlow
          .withVelocityX(-controller.getLeftY() * SwerveConstants.MAX_SPEED * SwerveConstants.BARGE_SPEED_MULTIPLIER)
          .withVelocityY(-controller.getLeftX() * SwerveConstants.MAX_SPEED * SwerveConstants.BARGE_SPEED_MULTIPLIER)
          .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));
  
          swerveCommandType = "SLOW";
          swerveCommandEntry.setString(swerveCommandType);
  
          break;
        default:
          dt.setControl(driveNormal
          .withVelocityX(-controller.getLeftY() * SwerveConstants.MAX_SPEED)
          .withVelocityY(-controller.getLeftX() * SwerveConstants.MAX_SPEED)
          .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));
  
          swerveCommandType = "NORMAL";
          swerveCommandEntry.setString(swerveCommandType);
          break;
      }

        break;
    }
    

    
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveCommandType = "INACTIVE";
    swerveCommandEntry.setString(swerveCommandType);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
