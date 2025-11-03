// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap.SafetyMap.SwerveConstants;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.posePathfindToReef;
import frc.robot.commands.swerve.posePathfindToReef.reefPole;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.GooseNeck;
import frc.robot.subsystems.GooseNeck.NeckState;
import frc.robot.subsystems.GooseNeckWheels;
import frc.robot.subsystems.GooseNeckWheels.WheelState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.vision.LimelightThreeBase.LEDState;
import frc.robot.subsystems.vision.LimelightThree3d;
import frc.robot.utils.LimelightPoseEstimate;

public class RobotContainer {
private CommandXboxController driverController = new CommandXboxController(0);
private CommandXboxController simController = new CommandXboxController(4);
private Elevator elevator = new Elevator();
private GooseNeck gooseNeck = new GooseNeck();
private GooseNeckWheels gooseNeckWheels = new GooseNeckWheels();
private LimelightThree3d limelight1 = new LimelightThree3d("limelight1", LEDState.OFF);
private LimelightThree3d limelight2 = new LimelightThree3d("limelight2", LEDState.OFF);
private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
private final Telemetry telemetry = new Telemetry(SwerveConstants.MAX_SPEED);
private SendableChooser<Command> autonChooser;
  public RobotContainer() {
    if(Robot.isSimulation()) configureSimControllerBindings();
    else configureBindings();
    drivetrain.registerTelemetry(telemetry::telemeterize);
    boolean isCompetition = true;
    autonChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("comp"))
          : stream
      );
  }

  private void configureBindings() {

    //Flash limelight LEDS when the robot starts to hold an algae
    new Trigger(gooseNeckWheels::holdingAlgae)
        .onTrue(limelight1.blinkLEDs(.75)
            .alongWith(limelight2.blinkLEDs(.75)));

    drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, elevator, driverController));
   
    // Align to right pole
    driverController.rightBumper().and(gooseNeckWheels::getCoralSeen)
        .whileTrue(new DeferredCommand(()-> new posePathfindToReef(reefPole.RIGHT, drivetrain), Set.of(drivetrain)))
    // Do nothing command to interrupt the alignment swerve request (returning to default command swerve control)
        .onFalse(new InstantCommand(()->{}, drivetrain));

    // Align to left pole
    driverController.leftBumper().and(gooseNeckWheels::getCoralSeen)
        .whileTrue(new DeferredCommand(()-> new posePathfindToReef(reefPole.LEFT, drivetrain), Set.of(drivetrain)))
    // Do nothing command to interrupt the alignment swerve request (returning to default command swerve control)
        .onFalse(new InstantCommand(()->{}, drivetrain));
    
    // Align to reef center (when robot doesn't have coral)
    driverController.leftBumper().and(()-> gooseNeckWheels.getCoralSeen())
    .or(driverController.rightBumper()).and(()-> gooseNeckWheels.getCoralSeen())
        .whileTrue(new DeferredCommand(()-> new posePathfindToReef(reefPole.CENTER, drivetrain), Set.of(drivetrain)))
        // Do nothing command to interrupt the alignment swerve request (returning to default command swerve control)
        .onFalse(new InstantCommand(()->{}, drivetrain));


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
                .andThen(gooseNeckWheels.infiniteStateCommand(WheelState.BACKTAKE)).until(()-> gooseNeckWheels.getCoralSeen())
                .andThen(elevator.stow())
                .alongWith(gooseNeckWheels.infiniteStateCommand(WheelState.BACKTAKE)).until(()-> gooseNeckWheels.getCoralSeen()));

    // Attempt to pick up ground algae on press (bound to right back button)
    driverController.povRight()
        .onTrue(elevator.moveToStateCommand(ElevatorState.ALGAE_GROUND)
            .andThen(gooseNeckWheels.algaeIn())
            .andThen(gooseNeck.moveToStateCommand(NeckState.ALGAE_GROUND_ANGLE)))
        .onFalse(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(new ConditionalCommand(gooseNeckWheels.algaeIn(), gooseNeckWheels.idle(), gooseNeckWheels::holdingAlgae)));
            
    // If Robot has a coral, shoot out the coral on press, then stow the elevator
    driverController.rightTrigger().and(gooseNeckWheels::getCoralSeen).and(()-> !gooseNeckWheels.holdingAlgae())
        .onTrue(gooseNeckWheels.score().deadlineFor(new WaitCommand(.3))
            .andThen(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE))
            .andThen(elevator.stow()));
    
    // If The robot is holding an algae, spit out the algae on press, then stow the elevator and gooseneck
    driverController.rightTrigger().and(gooseNeckWheels::holdingAlgae)
        .onTrue(gooseNeckWheels.algaeOut().raceWith(new WaitCommand(2))
            .andThen(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE))
            .andThen(elevator.stow().alongWith(gooseNeck.start())));

    // If robot has a coral, move to L4 position on press
    driverController.y().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L4))
            .andThen(gooseNeck.moveToStateCommand(NeckState.L4_ANGLE)));

    // If robot has algae, move to barge position on press
    driverController.y().and(gooseNeckWheels::holdingAlgae)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.BARGE_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.BARGE)));
    
    // If robot has algae, move to processor position on press
    driverController.a().and(gooseNeckWheels::holdingAlgae)
        .onTrue(gooseNeck.processor().alongWith(elevator.stow()));

    // If robot has a coral, move to L1 position on press
    driverController.a().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L1))
            .andThen(gooseNeck.moveToStateCommand(NeckState.L1_ANGLE)));

    // If robot has nothing, move to lower algae position on press
    driverController.b().and(()-> !gooseNeckWheels.getCoralSeen())
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.LOW_ALGAE))
            .andThen(gooseNeck.moveToStateCommand(NeckState.ALGAE_RETRIEVE_ANGLE)));

    // IF robot has coral, move to L2 position on press
    driverController.b().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L2)));
    
    // If robot has nothing, move to higher algae position on press
    driverController.x().and(()-> !gooseNeckWheels.getCoralSeen())
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.HIGH_ALGAE))
            .andThen(gooseNeck.moveToStateCommand(NeckState.ALGAE_RETRIEVE_ANGLE)));

    // If robot has coral, move to L2 position on press
    driverController.x().and(gooseNeckWheels::getCoralSeen)
        .onTrue(gooseNeck.moveToStateCommand(NeckState.STANDARD_ANGLE)
            .andThen(elevator.moveToStateCommand(ElevatorState.L3)));
    
    
  }

  private void configureSimControllerBindings(){
    drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, elevator, simController));
    // Align to right pole
    simController.rightBumper().whileTrue(new DeferredCommand(()-> new posePathfindToReef(reefPole.RIGHT, drivetrain), Set.of(drivetrain)))
    // Do nothing command to interrupt the alignment swerve request (returning to default command swerve control)
        .onFalse(new InstantCommand(()->{}, drivetrain));

    // Align to left pole
    simController.leftBumper().whileTrue(new DeferredCommand(()-> new posePathfindToReef(reefPole.LEFT, drivetrain), Set.of(drivetrain)))
    // Do nothing command to interrupt the alignment swerve request (returning to default command swerve control)
        .onFalse(new InstantCommand(()->{}, drivetrain));
    
    // Set elevator to barge state
    simController.y().onTrue(elevator.barge());
    // Set elevator to l4 state
    simController.b().onTrue(elevator.l4());
    // Set elevator to stow state
    simController.x().onTrue(elevator.stow());

  }

  /**
   * Method used to manage the addition of vision measurements to the robot's odometry pose.
   * @apiNote Should be called periodically (See Robot.java).
   */
  public void manageVisionMeasurements(){
    limelight1.setRobotOrientation(drivetrain.getState());
    LimelightPoseEstimate poseLL1 = limelight1.getRobotPose(true);
    if(limelight1.isPoseOk(poseLL1, drivetrain.getState())) drivetrain.addVisionMeasurement(poseLL1.getPose(), poseLL1.getTime());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
