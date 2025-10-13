// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TeleopSwerve extends Command {
  private CommandSwerveDrivetrain dt;
  private Elevator elevator;
  private CommandXboxController controller;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  
  private SwerveRequest.FieldCentric driveNormal;
  
  private SwerveRequest.FieldCentric driveSlow;

  private SwerveRequest.FieldCentric driveSlew;

  private GenericEntry swerveCommandEntry;
  private String swerveCommandType = "NORMAL";
  private final ShuffleboardTab tab = Shuffleboard.getTab(getName());

  /** Command used to control swerve in teleop. */
  public TeleopSwerve(CommandSwerveDrivetrain dt, Elevator elevator, CommandXboxController controller) {
    this.dt = dt;
    this.elevator = elevator;
    this.controller = controller;

    driveNormal = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);
    

    driveSlow = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);
    

    driveSlew = new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07);

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
