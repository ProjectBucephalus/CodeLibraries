// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Elevator.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class VerticalExtension extends SubsystemBase {
  private verticalState currentState = verticalState.ZERO;

    private final WPI_TalonFX verticalExtensionMotor = new WPI_TalonFX(Constants.kVerticalElevatorCanId);
  //  private final CANCoder verticalExtensionEncoder = new CANCoder(Constants.kVerticalElevatorEncoderCanId);
    private double armGoal = 0;

    //this subsystem uses a combination of a feedfoward and feedback control.
    //this gives us the advantage of better profiling from feedfoward and better precision from feedback.
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        Config.kVerticalExtensionKS, Config.kVerticalExtensionKG, //kS is the static friction constant, kG is the gravity constant
        Config.kVerticalExtensionKV, Config.kVerticalExtensionKA //kV is the velocity constant, kA is the acceleration constant
      );

  private static VerticalExtension myInstance;

  public static VerticalExtension getInstance()
  {
    if (myInstance == null)
    {
      myInstance = new VerticalExtension();
    }
    return myInstance;
  }
  /** Create a new ArmSubsystem. */
  private VerticalExtension() {

  }

  public enum verticalState{
    HIGH,
    SHELF,
    MEDIUM,
    LOW,
    HOME,
    ZERO,
    MIDHIGH,
  }

  public void setDesiredState(verticalState state){
    currentState = state;
  }
  public verticalState getState(){
    return currentState;
  }
  /** 
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    verticalExtensionMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    verticalExtensionMotor.config_kP(0, Config.kVerticalExtensionKP);
    verticalExtensionMotor.config_kD(0, Config.kVerticalExtensionKD);
    verticalExtensionMotor.config_kF(0, m_feedforward.calculate(Config.kVerticalExtensionMaxVelocity) / Config.kVerticalExtensionEncoderPPR);
    verticalExtensionMotor.config_kP(1, Config.kVerticalExtensionKP);
    verticalExtensionMotor.config_kD(1, Config.kVerticalExtensionKD);
    verticalExtensionMotor.config_kF( 1, m_feedforward.calculate(Config.kVerticalExtensionMaxVelocity) / Config.kVerticalExtensionEncoderPPR);
    verticalExtensionMotor.configMotionAcceleration(Config.kVerticalExtensionMaxAcceleration / (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR));
    verticalExtensionMotor.configMotionCruiseVelocity(Config.kVerticalExtensionMaxVelocity / (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR));
    verticalExtensionMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    verticalExtensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    verticalExtensionMotor.setNeutralMode(NeutralMode.Brake);
    
    verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    //verticalExtensionMotor.configRemoteFeedbackFilter(verticalExtensionEncoder, 0);
    //verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

 //   verticalExtensionEncoder.configFactoryDefault(); //reset and configure the encoder
   // verticalExtensionEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
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