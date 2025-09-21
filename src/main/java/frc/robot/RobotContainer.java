// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GooseNeck;
import frc.robot.subsystems.GooseNeckWheels;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.GooseNeck.NeckState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {
private CommandXboxController driverController = new CommandXboxController(0);
private Elevator elevator = new Elevator();
private GooseNeck gooseNeck = new GooseNeck();
private GooseNeckWheels gooseNeckWheels = new GooseNeckWheels();
private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
private final Telemetry telemetry = new Telemetry(SwerveConstants.MAX_SPEED);
  public RobotContainer() {
    configureBindings();
    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, elevator, driverController));
    
    driverController.rightTrigger().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeckWheels.score().raceWith(new WaitCommand(.3))
            .andThen(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE))
            .andThen(elevator.stow()));

    driverController.a()
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L4))
            .andThen(gooseNeck.moveToStateCommand(NeckState.L4_ANGLE)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
