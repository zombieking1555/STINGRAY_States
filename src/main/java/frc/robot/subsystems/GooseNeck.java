// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.GooseneckConstants;
import frc.robot.utils.SubsystemStatusManager;

public class GooseNeck extends SubsystemBase {

  public enum NeckState{
    START(Rotations.of(0)),
    STANDARD_ANGLE(Rotations.of(GooseneckConstants.STANDARDANGLE)), //Used for normal motion and L2/L3 Scoring
    INTAKE_ANGLE(Rotations.of(GooseneckConstants.INTAKEANGLE)),
    ALGAE_RETRIEVE_ANGLE(Rotations.of(GooseneckConstants.ALGAERETRIEVEANGLE)), //From the reef
    ALGAE_GROUND_ANGLE(Rotations.of(GooseneckConstants.ALGAEGROUNDANGLE)),
    L4_ANGLE(Rotations.of(GooseneckConstants.L4ANGLE)),
    L1_ANGLE(Rotations.of(GooseneckConstants.L1ANGLE)),
    BARGE_ANGLE(Rotations.of(GooseneckConstants.BARGEANGLE)),
    PROCESSOR_ANGLE(Rotations.of(GooseneckConstants.PROCESSORANGLE));

    private final Angle targetPosition;
    private final Angle tolerance;

    NeckState(Angle targetPosition){
      this.targetPosition = targetPosition;
      this.tolerance = Rotations.of(GooseneckConstants.tolerance);
    }

    public Angle getTargetPosition() {
        return targetPosition;
    }

    public Angle getTolerance() {
        return tolerance;
    }
  }

    private final TalonFX neckPivot = new TalonFX(GooseneckConstants.PIVOT_MOTOR);

    private final MotionMagicVoltage positionOut = new MotionMagicVoltage(Rotations.of(0));

    private NeckState currentState = NeckState.START;

    private final ShuffleboardTab tab = Shuffleboard.getTab(getName());
    private GenericEntry positionEntry; 
    private GenericEntry stateEntry;
    private GenericEntry atTargetEntry;

  public GooseNeck() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GooseneckConstants.SENSORTOMECHRATIO;
    config.CurrentLimits.StatorCurrentLimit = GooseneckConstants.PIVOT_MOTOR_CURRENT_LIMIT;
    
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    //Following values would need to be tuned.
    config.Slot0.kG = 0.0; // Constant applied for gravity compensation
    config.Slot0.kS = 0.0; // Constant applied for friction compensation (static gain)
    config.Slot0.kP = 0.0; // Proportional gain 
    config.Slot0.kD = 0.0; // Derivative gain
    config.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max allowed velocity (Motor rot / sec)
    config.MotionMagic.MotionMagicAcceleration = 0.0; // Max allowed acceleration (Motor rot / sec^2)
    // Apply config multiple times to ensure application
    for (int i = 0; i < 2; ++i){
      var status = neckPivot.getConfigurator().apply(config);
      if(status.isOK()) break;
    }

    positionEntry = tab.add("GooseNeck Position", 0.0).getEntry();
    stateEntry = tab.add("GooseNeck State", getCurrentState().name()).getEntry();
    atTargetEntry = tab.add("GooseNeck At Target", isAtTarget()).getEntry();

    SubsystemStatusManager.addSubsystem(getName(), ()-> neckPivot.isConnected());
}

@Override
public void periodic() {
  positionEntry.setDouble(getPosition().baseUnitMagnitude());
  stateEntry.setString(getCurrentState().name());
  atTargetEntry.setBoolean(isAtTarget());
  }

  public void setPosition(Angle position){
    neckPivot.setControl(positionOut.withPosition(position));
  }

  public void setState(NeckState state){
    currentState = state;
    setPosition(state.getTargetPosition());
  }

  public boolean isAtTarget(){
    return getPosition().isNear(getTargetPosition(), getTolerance());
  }

  public Angle getPosition(){
    return neckPivot.getPosition().getValue();
  }

  public Angle getTargetPosition(){
    return currentState.getTargetPosition();
  }

  public Angle getTolerance(){
    return currentState.getTolerance();
  }

  public void stop(){
    neckPivot.stopMotor();
  }

  public NeckState getCurrentState(){
    return currentState;
  }


  public Command start(){
    return runOnce(()-> setState(NeckState.START));
  }

  public Command groundAlgae(){
    return runOnce(()-> setState(NeckState.ALGAE_GROUND_ANGLE));
  }

  public Command retrieveAlgae(){
    return runOnce(()-> setState(NeckState.ALGAE_RETRIEVE_ANGLE));
  }

  public Command barge(){
    return runOnce(()-> setState(NeckState.BARGE_ANGLE));
  }

  public Command intake(){
    return runOnce(()-> setState(NeckState.INTAKE_ANGLE));
  }

  public Command l1(){
    return runOnce(()-> setState(NeckState.L1_ANGLE));
  }

  public Command l4(){
    return runOnce(()-> setState(NeckState.L4_ANGLE));
  }

  public Command processor(){
    return runOnce(()-> setState(NeckState.PROCESSOR_ANGLE));
  }

  public Command standard(){
    return runOnce(()-> setState(NeckState.STANDARD_ANGLE));
  }

  public Command moveToStateCommand(NeckState state){
    return new InstantCommand(()-> setState(state), this).andThen(new RunCommand(()-> {}, this).until(this::isAtTarget));
}
}
