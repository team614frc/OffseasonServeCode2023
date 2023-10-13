// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  CANSparkMax pivotMotor;

  public PivotSubsystem() {
    pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    
  }
  public void set(double val) {
    pivotMotor.set(val); 
  }
}
