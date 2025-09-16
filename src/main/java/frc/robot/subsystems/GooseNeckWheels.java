// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.events.CancelCommandEvent;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.GooseneckConstants;

public class GooseNeckWheels extends SubsystemBase {
  public enum WheelState{
    IDLE(0),
    INTAKE(GooseneckConstants.WHEEL_SPEED_INTAKE),
    SCORE(GooseneckConstants.WHEEL_SPEED_SCORE),
    L1(GooseneckConstants.L1_SPEED_SCORE),
    ALGAE_IN(GooseneckConstants.ALGAE_WHEEL_SPEED),
    ALGAE_OUT(GooseneckConstants.ALGAE_WHEEL_SPEED_SCORE);

    private final double output;
    WheelState(double output){
      this.output = output;
    }

    public double getOutput() {
        return output;
    }
  }

  private final TalonFX wheelMotor = new TalonFX(GooseneckConstants.INTAKE_MOTOR);

  private final CANrange coralRange = new CANrange(GooseneckConstants.CORALCANRANGE);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final ShuffleboardTab tab = Shuffleboard.getTab(getName());
  private GenericEntry stateEntry;

  private WheelState currentState = WheelState.IDLE;

  public GooseNeckWheels() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = GooseneckConstants.INTAKE_MOTOR_CURRENT_LIMIT;
    for (int i = 0; i < 2; ++i){
      var status = wheelMotor.getConfigurator().apply(config);
      if(status.isOK()) break;
    }
    stateEntry = tab.add("GooseNeck Wheels State", getCurrentState().name()).getEntry();
  }

  @Override
  public void periodic() {
    stateEntry.setString(getCurrentState().name());
  }

  public void setOutput(double output){
    wheelMotor.setControl(dutyCycleOut.withOutput(output));
  }

  public void setState(WheelState state){
    currentState = state;
    setOutput(state.getOutput());
  }

  public double getOutput(){
    return currentState.getOutput();
  }

 public boolean getCoralSeen(){
  return coralRange.getDistance().getValueAsDouble() < GooseneckConstants.CANRANGETHRESHOLDVALUE;
 }
 
  public WheelState getCurrentState(){
    return currentState;
  }

  public void stop(){
    wheelMotor.stopMotor();
  }


  public Command algaeIn(){
    return runOnce(()-> setState(WheelState.ALGAE_IN));
  }

  public Command algaeOut(){
    return runOnce(()-> setState(WheelState.ALGAE_OUT));
  }

  public Command idle(){
    return runOnce(()-> setState(WheelState.IDLE));
  }

  public Command intake(){
    return runOnce(()-> setState(WheelState.INTAKE));
  }

  public Command l1(){
    return runOnce(()-> setState(WheelState.L1));
  }

  public Command score(){
    return runOnce(()-> setState(WheelState.SCORE));
  }
}
