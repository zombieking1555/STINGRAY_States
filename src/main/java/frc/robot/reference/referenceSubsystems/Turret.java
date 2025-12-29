// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.reference.referenceSubsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DeviceTempReporter;
import frc.robot.utils.SubsystemStatusManager;

@Logged
public class Turret extends SubsystemBase {
  /** Creates a new Turret. 
   * Design notes:
   * Turret with approx. 420 deg. Rotation
   * Horizontally mounted limelight for apriltag tracking
   * External Cancoder geared up to 1:1.25
  */

  public enum TurretState {
    HOME(Rotations.of(0)),
    TRACKING(),
    UNWRAPPING();

    private Angle targetPos;
    private TurretState(Angle targetPos){
      this.targetPos = targetPos;
    }

    private TurretState(){
      targetPos = Rotations.of(0);
    }

    public Angle getTargetPosition(){
      return targetPos;
    }
  }

  private final TalonFX turretMotor = new TalonFX(1);
  private final CANcoder turretEncoder = new CANcoder(2);
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(Rotations.of(0));
  private final TurretLimelight limelight;

  private TurretState currentState = TurretState.HOME;

  private Angle currentPosition;
  private Angle positionError;
  private Angle targetPosition = Rotations.of(0);
  private boolean inWrapCycle = false;

  public Turret(TurretLimelight limelight) {
    this.limelight = limelight;
    CANcoderConfiguration cConfig = new CANcoderConfiguration();
    cConfig.MagnetSensor.MagnetOffset = 0; //Tune
    cConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0; //Tune
    // Apply config multiple times to ensure application
    for (int i = 0; i < 2; ++i){
      var status = turretEncoder.getConfigurator().apply(cConfig);
      if(status.isOK()) break;
    }

    TalonFXConfiguration tConfig = new TalonFXConfiguration();
    tConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tConfig.CurrentLimits.StatorCurrentLimit = 0; //Tune
    tConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    tConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    tConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //tConfig.Feedback. either need to use RotorToSensor or SensorToMechanism, maybe both, not quite sure.
    //Following values would need to be tuned.
    tConfig.Slot0.kS = 0.0; // Constant applied for friction compensation (static gain)
    tConfig.Slot0.kP = 0.0; // Proportional gain 
    tConfig.Slot0.kD = 0.0; // Derivative gain
    tConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0; // Max allowed velocity (Motor rot / sec)
    tConfig.MotionMagic.MotionMagicAcceleration = 0.0; // Max allowed acceleration (Motor rot / sec^2)
    // Apply config multiple times to ensure application
    for (int j = 0; j < 2; ++j){
      var status = turretMotor.getConfigurator().apply(tConfig);
      if(status.isOK()) break;
    }

    SubsystemStatusManager.addSubsystem(getName(), turretEncoder, turretMotor);
    DeviceTempReporter.addDevices(turretMotor);
  }

  @Override
  public void periodic() {
    currentPosition = getTurretAngle();
    positionError = limelight.getTurretRotationError();
   switch(currentState){
    case HOME: 
    setPosition(TurretState.HOME.getTargetPosition());
    break;
    case TRACKING:
      if(currentPosition.plus(positionError).abs(Rotations) > 1.05){
        setState(TurretState.UNWRAPPING);
        break;
      } 
      setPosition(currentPosition.plus(positionError));
      break;
    case UNWRAPPING:
      if(!inWrapCycle){ 
        if(getTurretAngle().gte(Rotations.of(0))){ 
          setPosition(currentPosition.minus(Rotations.of(1)));
        } else {
          setPosition(currentPosition.plus(Rotations.of(1)));
        }
      }
      inWrapCycle = true;

      if(getTurretAtSetpoint()){
        setState(TurretState.TRACKING);
        inWrapCycle = false;
      }
      break;
   }
   
  }
  public boolean getTurretAtSetpoint(){
    return getTurretAngle().isNear(targetPosition, 0.015);
  }

  public Angle getTurretAngle(){
    return turretMotor.getPosition().getValue();
  }

  public void setPosition(Angle posAngle){
    turretMotor.setControl(positionOut.withPosition(posAngle));
    targetPosition = posAngle;
  }
  
  /**
   * Enters a new state
   * @param state The state to enter
   */
  public void setState(TurretState state){
    currentState = state;
    setPosition(currentState.getTargetPosition());
  }
}
