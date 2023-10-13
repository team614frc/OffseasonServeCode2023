// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  CANSparkMax intakeMotorR;
  CANSparkMax intakeMotorL;
  
  public IntakeSubsystem() {
    // Creates a new motor
    intakeMotorR = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);
    intakeMotorR.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
    intakeMotorL = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
    intakeMotorL.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets the value of the motor to a double, at which the motor will run
  public void set(double val) {
    intakeMotorR.set(val);
    intakeMotorL.set(val);
  }

}
