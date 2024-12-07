// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Elevator.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Elevator.constants.Constants;
import frc.robot.Elevator.subsystems.Elevator.verticalState;

public class Pivot extends SubsystemBase {

  private TalonFX movementMotor = new TalonFX(Constants.kPivotCanId);
  
  /** Creates a new Pivot. */
  public Pivot() {}

  private double calculateMotorPosGoal(double manipAngleGoal) 
  {
    return 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
