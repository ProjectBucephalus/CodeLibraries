// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Elevator.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Elevator extends SubsystemBase {

  public void setDesiredState(verticalState state){
    currentState = state;
  }
  public verticalState getState(){
    return currentState;
  }
    resetSensors();
  }

   /**
   * Method to calculate the desired porition of the motor based off a target x and y position.
   * @param x desired x position
   * @param y desired y position
   * @return the desired setpoint for the extension in encoder units
   */
  public double calculateVerticalExtensionGoal(double x, double y) {
    return Config.kElevatorBaseWidth * Math.sin(Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) + Config.kVerticalExtensionPerpendicularHeight * Math.cos(Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight));//x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) * Config.kVerticalExtensionMetresPerRotation;
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
  if(getMeasurement() <= armGoal + Config.kVerticalExtensionPositionTolerenceMetres && getMeasurement() >= armGoal - Config.kVerticalExtensionPositionTolerenceMetres) {
      return true;
    }
    return false;
  }

  public boolean getIntakeLegal() {
    if(getMeasurement() <= 0) {
        return true;
      }
      return false;
    }
    public boolean getClawSafe() {
      if(getMeasurement() <= .1 && getMeasurement() >= -0.1) {
          return false;
        }
        return true;
      }
  /**
   * Get position of arm
   * @return vertical extension position in metres
   */
  public double getMeasurement() {
    return verticalExtensionMotor.getSelectedSensorPosition() / Config.kVerticalExtensionPulsesPerMetre;
  } 

  /**
   * Reset all sensors
   */
  public void resetSensors() {
    verticalExtensionMotor.setSelectedSensorPosition(0);
  }

  public boolean getLowLimit()
  {
    TalonFXSensorCollection sc = verticalExtensionMotor.getSensorCollection();
    return (sc.isRevLimitSwitchClosed() == 0);             //checks if the low limit switch is tripped
  }

public boolean getHighLimit()
{
  TalonFXSensorCollection sc = verticalExtensionMotor.getSensorCollection();
  return (sc.isFwdLimitSwitchClosed() == 0);               //checks if the high limit switch is tripped
}

private boolean lastLow = false;
private boolean lastHigh = false;

/**
 * Resets height if either top or bottom limit switches are activated.
 */
public void checkCalibration()
{
  boolean thisLow = getLowLimit();
  if ((thisLow == true) && (lastLow == false))
  {
    verticalExtensionMotor.setSelectedSensorPosition(Config.kArmBasePosY * Config.kVerticalExtensionPulsesPerMetre);
  }  
  lastLow = thisLow;
  boolean thisHigh = getHighLimit();
  if ((thisHigh == true) && (lastHigh == false))
  {
    verticalExtensionMotor.setSelectedSensorPosition(Config.kArmPeakPosY * Config.kVerticalExtensionPulsesPerMetre);
  }  
  lastHigh = thisHigh;
}

  /**
   * Move arm to desired position using motionMagic
   * @param position in metres
   */
  public void setPosition(double position) {
    // if(position > Config.kVerticalExtensionMaxHeight) { //Check if command is possible or not
    //   position = Config.kVerticalExtensionMaxHeight;
    // } else if(position <= 0) {
    //   position = 0;
    // }
    armGoal = position;
    verticalExtensionMotor.set(ControlMode.MotionMagic, position * Config.kVerticalExtensionPulsesPerMetre);
  }

    /**
   * Move arm to desired position using motionMagic
   * @param position in metres
   */
  public void setSpeed(double speed) {
    verticalExtensionMotor.set(ControlMode.PercentOutput, speed);
  }

}