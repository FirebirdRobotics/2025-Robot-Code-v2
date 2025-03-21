// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * References:
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/FusedCANcoder 
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // Need to configure CAN ID's
  private final TalonFX m_leader = new TalonFX(ElevatorConstants.elevatorLeaderMotorID, "CANivore");
  private final TalonFX m_follower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID, "CANivore");
  private final CANcoder m_CANcoder = new CANcoder(3, "CANivore");

  


  /** Creates a new Elevator. */
  public Elevator() {
    // Set follower motor to follow leader
    m_follower.setControl(new Follower(m_leader.getDeviceID(), false));

    CANcoderConfiguration CANCoderConfigs = new CANcoderConfiguration();
    // Need to Configure discontinuity point -- Should be as follows: (positive or negative) number of rotations taken to get max height of elevator
    // CANCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -1;
    // Need to configure depending on which direction encoder moves in while going upwards 
    CANCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


    var elevatorMotorConfigs = new TalonFXConfiguration();

    // Set's the feedback for the motor to be the cancoder
    // elevatorMotorConfigs.Feedback.FeedbackRemoteSensorID = m_CANcoder.getDeviceID();
    // elevatorMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elevatorMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // elevatorMotorConfigs.Feedback.SensorToMechanismRatio = 14.1428571429;
    // distance of the first stage off to 
    elevatorMotorConfigs.Feedback.SensorToMechanismRatio = ((19.65/7.75));

    elevatorMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;



    // set slot 0 gains
    var slot0Configs = elevatorMotorConfigs.Slot0;
    slot0Configs.kG = 0.32; // 
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 1.77; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 15; // A positio n error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0.01; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    


    

    
    // set Motion Magic settings
    var motionMagicConfigs = elevatorMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 30; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_leader.getConfigurator().apply(elevatorMotorConfigs);

  }

  // moves the elevator to intake height 
  public void goToHeight(double inches){

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    m_leader.setControl(m_request.withPosition(inches));
  }

public Command testElevator() {

    return runOnce(() -> goToHeight(4.5));


    
  }

  public Command goToDereefHigh() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.dereefHigh));

  }

  public Command goToDereefLow() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.dereefLow));

  }

  

  public Command goToL4() {
    
    return runOnce(() -> goToHeight(ElevatorConstants.L4height));

  }

  public Command goToL3() {
    return runOnce(() -> goToHeight(ElevatorConstants.L3height));
    
  }

  public Command goToL2() {
    return runOnce(() -> goToHeight(ElevatorConstants.L2height));
    
  }

  public Command goToL1() {
    return runOnce(() -> goToHeight(ElevatorConstants.L1height));

  }

  public Command goToStowedPosition() {
    return runOnce(() -> goToHeight(ElevatorConstants.stowedPosition));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // DogLog.log("Elevator position", m_leader.getPosition().getValueAsDouble());
    // DogLog.log("Elevator Velocity", m_leader.getVelocity().getValueAsDouble());
    // DogLog.log("Elevator Acceleration", m_leader.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Elevator position", m_leader.getPosition().getValueAsDouble());

  }
}
