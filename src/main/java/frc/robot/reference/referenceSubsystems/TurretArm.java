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
import frc.robot.reference.referenceSubsystems.Turret.TurretState;
import frc.robot.utils.SubsystemStatusManager;

@Logged
public class TurretArm extends SubsystemBase {
  public enum TurretArmState {
    HOME(Rotations.of(0)),
    TRACKING(),
    IDLE();

    private Angle targetPos;
    private TurretArmState(Angle targetPos){
      this.targetPos = targetPos;
    }

    private TurretArmState(){
      //target position of .25 rotations (Straight up)
      //This value should not be accessed, as position 
      //control of states without a preset setpoint should be managed in periodic().
      targetPos = Rotations.of(.25);
    }

    public Angle getTargetPosition(){
      return targetPos;
    }
  }


  private final TalonFX turretArmMotor = new TalonFX(3);
  private final CANcoder turretArmEncoder = new CANcoder(4);
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(Rotations.of(0));
  private final TurretLimelight limelight;

  private TurretArmState currentState = TurretArmState.HOME;

  private Angle currentPosition;
  private Angle positionError;
  private Angle targetPosition = Rotations.of(0);

  public TurretArm(TurretLimelight limelight) {
    this.limelight = limelight;
    CANcoderConfiguration cConfig = new CANcoderConfiguration();
    cConfig.MagnetSensor.MagnetOffset = 0; //Tune
    cConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0; //Tune
    // Apply config multiple times to ensure application
    for (int i = 0; i < 2; ++i){
      var status = turretArmEncoder.getConfigurator().apply(cConfig);
      if(status.isOK()) break;
    }

    TalonFXConfiguration tConfig = new TalonFXConfiguration();
    tConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tConfig.CurrentLimits.StatorCurrentLimit = 0; //Tune
    tConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    tConfig.Feedback.FeedbackRemoteSensorID = turretArmEncoder.getDeviceID();
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
      var status = turretArmMotor.getConfigurator().apply(tConfig);
      if(status.isOK()) break;
    }

    SubsystemStatusManager.addSubsystem(getName(), turretArmEncoder, turretArmMotor);
  }

  @Override
  public void periodic() {
    currentPosition = getTurretArmAngle();
    positionError = limelight.getTurretArmRotationError();
   switch(currentState){
    case HOME: 
    setPosition(TurretState.HOME.getTargetPosition());
    break;
    case TRACKING:
      if(targetPosition.gt(Rotations.of(0)) || targetPosition.lt(Rotations.of(.5))){
        setState(TurretArmState.IDLE);
        break;
      } 
      setPosition(currentPosition.plus(positionError));
      break;
    case IDLE:
      turretArmMotor.set(0);
    break;
   }
   
  }

  public boolean getTurretAtSetpoint(){
    return getTurretArmAngle().isNear(targetPosition, 0.015);
  }

  public Angle getTurretArmAngle(){
    return turretArmMotor.getPosition().getValue();
  }

  public void setPosition(Angle posAngle){
    turretArmMotor.setControl(positionOut.withPosition(posAngle));
    targetPosition = posAngle;
  }
  
  /**
   * Enters a new state
   * @param state The state to enter
   */
  public void setState(TurretArmState state){
    currentState = state;
  }
}
