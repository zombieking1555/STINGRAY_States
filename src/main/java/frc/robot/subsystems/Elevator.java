// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ElevatorMap;
import frc.robot.utils.SubsystemStatusManager;

public class Elevator extends SubsystemBase {
  
  public enum ElevatorState {

    STOW(Rotations.of(ElevatorMap.ELEVATOR_LIMIT_SWITCH_HEIGHT)),
    //This works kinda like a key and gives it proprityes
    ALGAE_GROUND(Rotations.of(ElevatorMap.ALGAE_GROUND_INTAKE_ROTATION)),
    //There are constructos right next to the thing
    CORAL_INTAKE(Rotations.of(ElevatorMap.CORAL_INTAKE_ROTATION)),
    L1(Rotations.of(ElevatorMap.L1ROTATION)),
    //Rotations.of is taking the distance from 0 to L1
    //Syntax = item()
    L2(Rotations.of(ElevatorMap.L2ROTATION)),
    L3(Rotations.of(ElevatorMap.L3ROTATION)),
    L4(Rotations.of(ElevatorMap.L4ROTATION)),
    HIGH_ALGAE(Rotations.of(ElevatorMap.HIGHALGAEROTATION)),
    LOW_ALGAE(Rotations.of(ElevatorMap.LOWALGAEROTATION)),
    BARGE(Rotations.of(ElevatorMap.BARGEROTATION));

    private final Angle targetPosition;
    private final Angle tolerance;

    ElevatorState(Angle targetPosition){
      this.targetPosition = targetPosition;
      this.tolerance = Rotations.of(.2);
    }

    public Angle getTargetPosition() {
        return targetPosition;
    }

    public Angle getTolerance() {
        return tolerance;
    }
  }

  private final TalonFX elevatorLeader = new TalonFX(ElevatorMap.ELEVATOR_LEADER);
  private final TalonFX elevatorFollower = new TalonFX(ElevatorMap.ELEVATOR_FOLLOWER);
  private final DigitalInput elevatorSwitch = new DigitalInput(ElevatorMap.LIMIT_SWITCH_CHANNEL);

  private boolean lastLimitSwitchState = false;
  private long lastLimitSwitchTimeMs = 0;
  private static final long LIMIT_SWITCH_DEBOUNCE_MS = 100;

  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(Rotations.of(0));

  private ElevatorState currentState = ElevatorState.STOW;

  private final ShuffleboardTab tab = Shuffleboard.getTab(getName());
  private GenericEntry positionEntry; 
  private GenericEntry stateEntry;
  private GenericEntry atTargetEntry;
  
  public Elevator() {
    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = ElevatorMap.ELEVATOR_CURRENT_LIMIT;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    //Following values would need to be tuned.
    config.Slot0.kG = 0.0; // Constant applied for gravity compensation
    config.Slot0.kS = 0.0; // Constant applied for friction compensation (static gain)
    config.Slot0.kP = 0.0; // Proportional gain 
    config.Slot0.kD = 0.0; // Derivative gain
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max allowed velocity (Motor rot / sec)
    config.MotionMagic.MotionMagicAcceleration = 0.0; // Max allowed acceleration (Motor rot / sec^2)
    // Apply config multiple times to ensure application
    for (int i = 0; i < 2; ++i){
      var status = elevatorLeader.getConfigurator().apply(config);
      if(status.isOK()) break;
    }
    //initializing shuffleboard widget once so that the entry it contains can be updated in periodic (less expensive)
    positionEntry = tab.add("Elevator Position", 0.0).getEntry();
      stateEntry = tab.add("elevatorState", getCurrentState().name()).getEntry();
      atTargetEntry = tab.add("elevatorAtTarget", isAtTarget()).getEntry();

      SubsystemStatusManager.addSubsystem(getName(), elevatorLeader, elevatorFollower);
  }

  @Override
  public void periodic() {
    positionEntry.setDouble(getPosition().baseUnitMagnitude());
    stateEntry.setString(getCurrentState().name());
    atTargetEntry.setBoolean(isAtTarget());


    //Limit switch logic to reset position of the leader motor on a debounce as to not spam
    //the motor with position resets if it isn't necessary.
    boolean currentLimitSwitchState = !elevatorSwitch.get();
    long timeNow = System.currentTimeMillis();

    if(currentLimitSwitchState != lastLimitSwitchState){
      lastLimitSwitchTimeMs = timeNow;
      lastLimitSwitchState = currentLimitSwitchState;

    } else if((timeNow -lastLimitSwitchTimeMs) > LIMIT_SWITCH_DEBOUNCE_MS){
      if(currentLimitSwitchState){
        elevatorLeader.setPosition(Rotations.of(ElevatorMap.ELEVATOR_LIMIT_SWITCH_HEIGHT));
      }
    }
  }

  public void setPosition(Angle position){
    elevatorLeader.setControl(positionOut.withPosition(position));
  }

  public void setState(ElevatorState state){
    currentState = state;
    setPosition(state.getTargetPosition());
  }

  public boolean isAtTarget(){
    return getPosition().isNear(getTargetPosition(), getTolerance());
  }

  public boolean isReefAlgaeState(){
    return getCurrentState() == ElevatorState.HIGH_ALGAE || 
           getCurrentState() == ElevatorState.LOW_ALGAE;
  }

  public Angle getPosition(){
    return elevatorLeader.getPosition().getValue();
  }

  public Angle getTargetPosition(){
    return currentState.getTargetPosition();
  }

  public Angle getTolerance(){
    return currentState.getTolerance();
  }

  public void stop(){
    elevatorLeader.stopMotor();
  }

  public ElevatorState getCurrentState(){
    return currentState;
  }

  public boolean isFunctional(){
    return elevatorLeader.isConnected() && elevatorFollower.isConnected();
  }

public Command stow(){
  return runOnce(()-> setState(ElevatorState.STOW));
  //()-> just means that it is runable
}
public Command coralIntake(){
  return runOnce(()-> setState(ElevatorState.CORAL_INTAKE));
}
public Command algaeGround(){
  return runOnce(()-> setState(ElevatorState.ALGAE_GROUND));
}
public Command l1(){
  return runOnce(()-> setState(ElevatorState.L1));
}
public Command l2(){
  return runOnce(()-> setState(ElevatorState.L2));
}
public Command l3(){
  return runOnce(()-> setState(ElevatorState.L3));
}
public Command l4(){
  return runOnce(()-> setState(ElevatorState.L4));
}
public Command highAlgae(){
  return runOnce(()-> setState(ElevatorState.HIGH_ALGAE));
}
public Command lowAlgae(){
  return runOnce(()-> setState(ElevatorState.LOW_ALGAE));
}
public Command barge(){
  return runOnce(()-> setState(ElevatorState.BARGE));
}

/**Command that drives to a state and finishes when at the target*/
public Command moveToStateCommand(ElevatorState state){
    return new InstantCommand(()-> setState(state), this).andThen(new RunCommand(()-> {}, this).until(this::isAtTarget));
}

}

