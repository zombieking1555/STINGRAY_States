// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.reference.referenceSubsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialWrist extends SubsystemBase {
  /** Creates a new DifferentialElevator. */
  private TalonFX motorAdd = new TalonFX(1);
  private TalonFX motorSub = new TalonFX(2);
  private CANcoder diffCancoder = new CANcoder(3);
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private PositionVoltage avg = new PositionVoltage(Rotations.of(0));
  private PositionVoltage diff = new PositionVoltage(Rotations.of(0));
  // differential axis: wrist rotate, primary axis: wrist up/down
  private DifferentialMechanism differentialWrist = new DifferentialMechanism(motorSub, motorAdd, false, diffCancoder);
  private DifferentialConstantsConfigs diffConfigs = new DifferentialConstantsConfigs();

  public DifferentialWrist() {
    diffConfigs.PeakDifferentialVoltage = 10.0;

     // Primary PID
     motorConfig.Slot0.kP = 6.0; 
     motorConfig.Slot0.kI = 0.0;
     motorConfig.Slot0.kD = 0.2;
     
 
     // Differential PID
     motorConfig.Slot1.kP = 8.0; 
     motorConfig.Slot1.kI = 0.0;
     motorConfig.Slot1.kD = 0.25;

     motorConfig.withDifferentialConstants(diffConfigs);
     
     motorAdd.getConfigurator().apply(motorConfig);
     motorSub.getConfigurator().apply(motorConfig);

     differentialWrist.applyConfigs();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWristDifferentialPosition(Angle avg, Angle diff){
    differentialWrist.setControl(this.avg.withPosition(avg), this.diff.withPosition(diff));
  }

  public void commandExample(DifferentialMechanism differentialWrist) {
    // Primary: 0 rotations (wrist up/down = 0)
    PositionVoltage avgReq = new PositionVoltage(Rotations.of(0.0))
        .withSlot(0); // use Slot0 gains for the primary loop

    // Differential: 180 degrees -> 0.5 rotations
    PositionVoltage diffReq = new PositionVoltage(Rotations.of(0.5))
        .withSlot(1); // use Slot1 gains for the differential loop

    // send command
    differentialWrist.setControl(avgReq, diffReq);
  }
}
