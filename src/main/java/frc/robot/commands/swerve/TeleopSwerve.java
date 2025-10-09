// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class TeleopSwerve extends Command {
  private CommandSwerveDrivetrain dt;
  private Elevator elevator;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.L4_SLEW_RATE);
  
  private Command driveNormal;
  
  private Command driveSlow;

  private Command driveSlew;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(CommandSwerveDrivetrain dt, Elevator elevator, CommandXboxController controller) {
    this.dt = dt;
    this.elevator = elevator;

    driveNormal =  dt.applyRequest(()-> new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07)
    .withVelocityX(-controller.getLeftY() * SwerveConstants.MAX_SPEED)
    .withVelocityY(-controller.getLeftX() * SwerveConstants.MAX_SPEED)
    .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));

    driveSlow =  dt.applyRequest(()-> new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07)
    .withVelocityX(-controller.getLeftY() * SwerveConstants.MAX_SPEED * SwerveConstants.BARGE_SPEED_MULTIPLIER)
    .withVelocityY(-controller.getLeftX() * SwerveConstants.MAX_SPEED * SwerveConstants.BARGE_SPEED_MULTIPLIER)
    .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));

    driveSlew =  dt.applyRequest(()-> new SwerveRequest.FieldCentric()
    .withDeadband(SwerveConstants.MAX_SPEED*.07)
    .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE*.07)
    .withVelocityX(xLimiter.calculate(-controller.getLeftY() * SwerveConstants.MAX_SPEED))
    .withVelocityY(yLimiter.calculate(-controller.getLeftX() * SwerveConstants.MAX_SPEED))
    .withRotationalRate(-controller.getRightX() * SwerveConstants.MAX_ANGULAR_RATE));

    addRequirements(this.dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command toSchedule = driveNormal;

    switch(elevator.getCurrentState()){
      case L4:
        toSchedule = driveSlew;
        break;
      case BARGE:
        toSchedule = driveSlow;
        break;
      default:
        toSchedule = driveNormal;
        break;
    }

    CommandScheduler scheduler = CommandScheduler.getInstance();
    if (!scheduler.isScheduled(toSchedule)) {
      // cancel other drive variants (so only one holds the drivetrain)
      if (scheduler.isScheduled(driveNormal) && driveNormal != toSchedule) scheduler.cancel(driveNormal);
      if (scheduler.isScheduled(driveSlew) && driveSlew != toSchedule) scheduler.cancel(driveSlew);
      if (scheduler.isScheduled(driveSlow) && driveSlow != toSchedule) scheduler.cancel(driveSlow);

      scheduler.schedule(toSchedule);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
  if (scheduler.isScheduled(driveNormal)) scheduler.cancel(driveNormal);
  if (scheduler.isScheduled(driveSlew))  scheduler.cancel(driveSlew);
  if (scheduler.isScheduled(driveSlow))  scheduler.cancel(driveSlow);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
