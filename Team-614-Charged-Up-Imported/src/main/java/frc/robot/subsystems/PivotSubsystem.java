// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The PivotSubsystem has the motor objects for the motors of the
 * Pivot on the robot. It also sets them a value based on the input
 * received from a command
 * - 
 * @param pivotSpeed Variable represents the speed passed from a command
 * that pivot motors should be set to
 * @returns pivotPosition
 */

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private CANSparkMax pivotMotor;

  public PivotSubsystem() {
    pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPivotMotorHeight () {
    return pivotMotor.getEncoder().getPosition();
  }

  public void set(double pivotSpeed) {
    pivotMotor.set(pivotSpeed); 
  }


}
