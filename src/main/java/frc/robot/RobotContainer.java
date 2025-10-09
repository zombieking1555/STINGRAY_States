// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GooseNeck;
import frc.robot.subsystems.GooseNeckWheels;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.GooseNeck.NeckState;
import frc.robot.subsystems.GooseNeckWheels.WheelState;
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
    
    // If Robot is in a state to pick up algae, spin wheels to intake an algae on press
    driverController.leftTrigger().and(elevator::isReefAlgaeState)
        .onTrue(gooseNeckWheels.algaeIn())
        .onFalse(new ConditionalCommand(gooseNeckWheels.algaeIn(), gooseNeckWheels.idle(), gooseNeckWheels::holdingAlgae));
    
    // If Robot isn't in a state to pick up algae, spin wheels to intake a coral on press
    driverController.leftTrigger().and(()-> !elevator.isReefAlgaeState())
        .onTrue(elevator.coralIntake().alongWith(gooseNeck.intake())
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.INTAKE)).until(()-> gooseNeckWheels.getCoralSeen())
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.INTAKE)).until(()-> !gooseNeckWheels.getCoralSeen())
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.BACKTAKE)).until(()-> gooseNeckWheels.getCoralSeen())
                //Initial sequence done, now an extra check to ensure coral placement is correct
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.INTAKE)).until(()-> !gooseNeckWheels.getCoralSeen())
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.BACKTAKE)).until(()-> gooseNeckWheels.getCoralSeen()));

    // Attempt to pick up ground algae on press (bound to right back button)
    driverController.povRight()
        .onTrue(elevator.moveToStateCommand(ElevatorState.ALGAE_GROUND)
            .andThen(gooseNeckWheels.algaeIn())
            .andThen(gooseNeck.moveToStateCommand(NeckState.ALGAE_GROUND_ANGLE)))
        .onFalse(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(new ConditionalCommand(gooseNeckWheels.algaeIn(), gooseNeckWheels.idle(), gooseNeckWheels::holdingAlgae)));
            
    // If Robot has a coral, shoot out the coral on press, then stow the elevator
    driverController.rightTrigger().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeckWheels.score().raceWith(new WaitCommand(.3))
            .andThen(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE))
            .andThen(elevator.stow()));
    
    // If The robot is holding an algae, spit out the algae on press, then stow the elevator
    driverController.rightTrigger().and(gooseNeckWheels::holdingAlgae)
        .onTrue(gooseNeckWheels.algaeOut().raceWith(new WaitCommand(2))
            .andThen(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE))
            .andThen(elevator.stow()));

    // If robot has a coral, move to L4 position on press
    driverController.a().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L4))
            .andThen(gooseNeck.moveToStateCommand(NeckState.L4_ANGLE)));

    // If robot has algae, move to barge position on press
    driverController.a().and(gooseNeckWheels::holdingAlgae)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.BARGE_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.BARGE)));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
