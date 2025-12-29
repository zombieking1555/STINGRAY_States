// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAlign extends Command {
  private PIDController xPid = new PIDController(0.1, 0, 0);
  private PIDController yPid = new PIDController(0.1, 0, 0);
  private PIDController rotPid = new PIDController(0.1, 0, 0);
  private double goalX;
  private double goalY;
  private double goalRot;
  private CommandSwerveDrivetrain dt;
  private SwerveRequest.FieldCentric control = new SwerveRequest.FieldCentric();
  /** Creates a new autoAlign. */
  public autoAlign(Pose2d targetPose, CommandSwerveDrivetrain dt) {
    this.goalX = targetPose.getX();
    this.goalY = targetPose.getY();
    this.goalRot = targetPose.getRotation().getDegrees();
    this.dt = dt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPid.setSetpoint(goalX);
    yPid.setSetpoint(goalY);
    rotPid.setSetpoint(goalRot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = xPid.calculate(dt.getState().Pose.getX()), 
      yOutput = yPid.calculate(dt.getState().Pose.getY()), 
      rotOutput = rotPid.calculate(dt.getState().Pose.getRotation().getDegrees());
    dt.setControl(
        control.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rotOutput)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
