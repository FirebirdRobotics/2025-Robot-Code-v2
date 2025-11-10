// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, "CANivore");
  private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, "CANivore");
  private final CANrange intakeCANrange = new CANrange(45, "CANivore");

  /** Creates a new Intake. */
  public Intake() {

    var intakeCANRangeConfigs = new CANrangeConfiguration();
    intakeCANRangeConfigs.ProximityParams.ProximityThreshold = 0.3;

    var pivotMotorConfigs = new TalonFXConfiguration();

    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotMotorConfigs.Feedback.SensorToMechanismRatio = 23.265306122;

    pivotMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    // Set Slot 0 Configurations
    var slot0Configs = pivotMotorConfigs.Slot0;
    slot0Configs.kG = 0.0; //
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.46000000834465027; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 20; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.00001; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    // set Motion Magic settings
    var motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30.0; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 30.0; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_pivotMotor.getConfigurator().apply(pivotMotorConfigs);

    var intakeRollerConfigs = new TalonFXConfiguration();
    intakeRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeRollerConfigs.CurrentLimits.StatorCurrentLimit = 50;

    intakeRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeRollerConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    m_rollerMotor.getConfigurator().apply(intakeRollerConfigs);
  }

  public void goToAngle(double angle) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(angle);
    m_pivotMotor.setControl(m_request.withPosition(angle));
  }

  public void goToDeployedPosition() {
    goToAngle(IntakeConstants.DEPLOY_ANGLE);
    setRollerMotorPercentOutput(0.6);
  }

  public void goToFramePerimeterPosition() {
    goToAngle(IntakeConstants.FRAME_PERIMETER_ANGLE);
    setRollerMotorPercentOutput(0.6);
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    m_rollerMotor.setControl(new DutyCycleOut(outputPercent));
  }

  @Override
  public void periodic() {
    // Runs every robot cycle (20ms)
  }
}
