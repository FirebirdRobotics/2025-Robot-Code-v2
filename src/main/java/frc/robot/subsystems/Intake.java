// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(13, "CANivore");
  private final TalonFX m_rollerMotor = new TalonFX(14, "CANivore");
  private final CANrange intakeCANrange = new CANrange(20, "CANivore");


  /** Creates a new Intake. */
  public Intake() {
    var pivotMotorConfigs = new TalonFXConfiguration();

    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotMotorConfigs.Feedback.SensorToMechanismRatio = 23.265306122;
    


    // set slot 0 gains
    var slot0Configs = pivotMotorConfigs.Slot0;
    slot0Configs.kG = 0.0; // 
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.46000000834465027; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 20; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.00001; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    // need to 
    // var positionVoltageCongigs = pivotMotorConfigs.

    // set Motion Magic settings
    // var motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 0.0; // Target cruise velocity of 80 rps
    // motionMagicConfigs.MotionMagicAcceleration = 0.0; // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_pivotMotor.getConfigurator().apply(pivotMotorConfigs);

    
  }

  // 0 for starting
  // -0.243 for deployed
  // Intake Deploys at inputed angle
  public void goToAngle(double angle) {
    // final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    final PositionVoltage m_request = new PositionVoltage(angle).withSlot(0);

    m_pivotMotor.setControl(m_request.withPosition(angle));
  }

  public void goToDeployedPosition() {
    
  }

  /* */
  public Command CommandGoToAngle(double angle) {
    return new InstantCommand(()-> goToAngle(angle), this);
  }
  

  
  public void setRollerMotorPercentOutput(double outputPercent) {
    m_rollerMotor.setControl(new DutyCycleOut(outputPercent));
  }

  public Command setRollerMotorPercentOutputCommand(double power) {
    
    return runEnd(
      () -> setRollerMotorPercentOutput(0.442),
      () -> setRollerMotorPercentOutput(0)
    ); 

  }


  // public boolean motorHitHardstop() {

  // }

  /**
  @return true if detects coral, false if it doesn't
  */
  public boolean getIntakeCANrange() {
    // Max distance away from sensor coral can be while still considering it "detected" distance units is meters
    double maxDetectableDistanceMeters = 0.3;
    if (intakeCANrange.getDistance().getValueAsDouble() <= maxDetectableDistanceMeters) {
      return true;
    }
    return false;
  }

  // public Command runIntakeRollersUntilCANrange() {
  //   return new ParallelDeadlineGroup(null, null)
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeCANRangeDistance", intakeCANrange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Angle", m_pivotMotor.getPosition().getValueAsDouble());
    DogLog.log("Intake/IntakeCANrangeDistance", intakeCANrange.getDistance().getValueAsDouble());
    DogLog.log("Intake/IntakePivotAngle", m_pivotMotor.getPosition().getValueAsDouble());
  }
}
