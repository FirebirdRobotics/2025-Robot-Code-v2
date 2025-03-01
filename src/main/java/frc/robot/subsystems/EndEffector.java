// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class EndEffector extends SubsystemBase {
    // Need to configure CAN ID's
  private final TalonFX m_endEffectorPivot = new TalonFX(7, "canivore");
  private final CANcoder m_CANcoder = new CANcoder(8, "canivore");

  CANrange endEffectorCANrange = new CANrange(9);

  /** Creates a new EndEffector. */
  public EndEffector() {

    CANcoderConfiguration CANCoderConfigs = new CANcoderConfiguration();
    
    // Need to Configure discontinuity point -- Should be as follows: (positive or negative) number of rotations taken to get max height of elevator
    CANCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -1;
    
    // Need to configure depending on which direction encoder moves in while going upwards 
    CANCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    var endEffectorPivotConfigs = new TalonFXConfiguration();

    // Set's the feedback for the motor to be the cancoder
    endEffectorPivotConfigs.Feedback.FeedbackRemoteSensorID = m_CANcoder.getDeviceID();
    endEffectorPivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    endEffectorPivotConfigs.Feedback.SensorToMechanismRatio = 0.0;



    // set slot 0 gains
    var slot0Configs = endEffectorPivotConfigs.Slot0;
    slot0Configs.kG = 0.0; // 
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.0; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
  }

  public boolean getEndEffectorCANrange() {
    // Max distance away from sensor coral can be while still considering it "detected" distance units is meters
    double maxDetectableDistanceMeters = 0.01;
    if (endEffectorCANrange.getDistance().getValueAsDouble() <= maxDetectableDistanceMeters) {
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
