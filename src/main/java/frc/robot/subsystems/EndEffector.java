// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class EndEffector extends SubsystemBase {
    // Need to configure CAN ID for roller
  private final TalonFX m_endEffectorRollers = new TalonFX(40, "CANivore");
  
  private final TalonFX m_endEffectorPivot = new TalonFX(47, "CANivore");
  private final CANcoder m_CANcoder = new CANcoder(43, "CANivore");

  private final CANrange endEffectorOuterCANrange = new CANrange(50, "CANivore");
  private final CANrange endEffectorInnerCANrange = new CANrange(51, "CANivore");


  LEDs m_LEDs;
  /** Creates a new EndEffector. */
  public EndEffector(LEDs leds) {
    m_LEDs = leds;
    var endEffectorOuterCANrangeConfigs = new CANrangeConfiguration();
    var endEffectorInnerCANrangeConfigs = new CANrangeConfiguration();

    // Max distance away from sensor coral can be while still considering it "detected" distance units is meters
    endEffectorOuterCANrangeConfigs.ProximityParams.ProximityThreshold = 0.1;
    endEffectorInnerCANrangeConfigs.ProximityParams.ProximityThreshold = 0.1;

    endEffectorOuterCANrange.getConfigurator().apply(endEffectorOuterCANrangeConfigs);
    endEffectorInnerCANrange.getConfigurator().apply(endEffectorInnerCANrangeConfigs);

    


    CANcoderConfiguration CANCoderConfigs = new CANcoderConfiguration();
    
    // Need to Configure discontinuity point -- Should be as follows: (positive or negative) number of rotations taken to get max height of elevator
    // CANCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -1;
    
    // Need to configure depending on which direction encoder moves in while going upwards 
    CANCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    var endEffectorPivotConfigs = new TalonFXConfiguration();

    // Set's the feedback for the motor to be the cancoder
    endEffectorPivotConfigs.Feedback.FeedbackRemoteSensorID = m_CANcoder.getDeviceID();
    endEffectorPivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    endEffectorPivotConfigs.Feedback.SensorToMechanismRatio = 1;

    endEffectorPivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    endEffectorPivotConfigs.CurrentLimits.StatorCurrentLimit = 50;

    endEffectorPivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    endEffectorPivotConfigs.CurrentLimits.SupplyCurrentLimit = 50;




    // set slot 0 gains
    var slot0Configs = endEffectorPivotConfigs.Slot0;
    slot0Configs.kG = 0.0; // 
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 3.9000000953674316; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 22; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = endEffectorPivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 5; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    var endEffectorRollerConfigs = new TalonFXConfiguration();
    endEffectorRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    endEffectorRollerConfigs.CurrentLimits.StatorCurrentLimit = 50;

    endEffectorRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    endEffectorRollerConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    // Set m_endEffectorPivot to brake mode
    m_endEffectorPivot.setNeutralMode(NeutralModeValue.Coast);

    m_endEffectorRollers.getConfigurator().apply(endEffectorRollerConfigs);




  }

    public void goToAngle(double angle) {
    // final PositionVoltage m_request = new PositionVoltage(IntakeConstants.deployAngle).withSlot(0);

    // m_pivotMotor.setControl(m_request.withPosition(IntakeConstants.deployAngle));

    final MotionMagicVoltage m_request = new MotionMagicVoltage(angle);

    m_endEffectorPivot.setControl(m_request.withPosition(angle));
    

  }

  public Command testEndEffector(double angle) {
    return runOnce(() -> goToAngle(angle));
  }

  public Command goToDereefHigh() {
    
    return runOnce(() -> goToAngle(EndEffectorConstants.dereefHigh));

  }

  public Command goToDereefLow() {
    
    return runOnce(() -> goToAngle(EndEffectorConstants.dereefLow));

  }

  


  public Command goToL4() {
    return runOnce(() -> goToAngle(EndEffectorConstants.L4Angle));

  }

  public Command goToL3() {
    return runOnce(() -> goToAngle(EndEffectorConstants.L3Angle));
  }

  public Command goToL2() {
    return runOnce(() -> goToAngle(EndEffectorConstants.L2Angle));

  }

  public Command goToL1() {
    return runOnce(() -> goToAngle(EndEffectorConstants.L1Angle));

  }

  public Command goToStowed() {
      return runOnce(() -> goToAngle(EndEffectorConstants.stowedAngle));
    }

  public Command goToParty() {
    return Commands.repeatingSequence(
        Commands.sequence(
            runOnce(() -> {
                goToAngle(EndEffectorConstants.partyForward);
                m_LEDs.setRed();
            }),
            Commands.waitSeconds(0.7),
            runOnce(() -> {
                goToAngle(EndEffectorConstants.stowedAngle);
                m_LEDs.setBlack();
            }),
            runOnce(() -> {
                goToAngle(EndEffectorConstants.partyBackward);
                m_LEDs.setRed();
            }),
            Commands.waitSeconds(0.7),
            runOnce(() -> {
                goToAngle(EndEffectorConstants.partyBackward);
                m_LEDs.setBlack();
            })
        )
    );
  }

  private boolean isPartyActive = false;

  public Command toggleParty() {
      return runOnce(() -> {
          if (isPartyActive) {
              CommandScheduler.getInstance().cancelAll(); // Stop the cycle
              isPartyActive = false;
          } else {
              CommandScheduler.getInstance().schedule(goToParty()); // Start the cycle
              isPartyActive = true;
          }
      });
  }

  public boolean getendEffectorOuterCANrange() {
  
    return endEffectorOuterCANrange.getIsDetected().getValue();
  }

  public boolean getendEffectorInnerCANrange() {
  
    return endEffectorInnerCANrange.getIsDetected().getValue();
  }


  

    public void setRollerMotorPercentOutput(double outputPercent) {
      m_endEffectorRollers.setControl(new DutyCycleOut(outputPercent));
  }

  
  public Command setRollerMotorPercentOutputAndThenTo0Command(double power) {
    
    return runEnd(
      () -> setRollerMotorPercentOutput(power),
      () -> setRollerMotorPercentOutput(0)
    ); 

  }

  public Command setRollerMotorPercentOutputCommand(double power) {

    return runOnce(() -> setRollerMotorPercentOutput(power));

  }

  // public void juggleCoralTillBothCanRangesTrue() {
  //   if (!(getendEffectorInnerCANrange() && (getendEffectorOuterCANrange()))) {
  //     if (getendEffectorInnerCANrange() && (getendEffectorOuterCANrange() == false)) {
  //       setRollerMotorPercentOutput(-0.2);
  //     }
  //     if ((getendEffectorInnerCANrange() == false) && (getendEffectorOuterCANrange())) {
  //       setRollerMotorPercentOutput(0.2);
  //     }
  
  //   }

  //   if (getendEffectorInnerCANrange() && (getendEffectorOuterCANrange() == false)) {
  //     setRollerMotorPercentOutput(-0.2);
  //   }
  //   if ((getendEffectorInnerCANrange() == false) && (getendEffectorOuterCANrange())) {
  //     setRollerMotorPercentOutput(0.2);
  //   }

  // }





  @Override
  public void periodic() {
    if (getendEffectorInnerCANrange() && getendEffectorOuterCANrange()) {
      m_LEDs.setRed();
    }



    // This method will be called once per scheduler run
    // DogLog.log("EndEffector/End Effector rotations", m_endEffectorPivot.getPosition().getValueAsDouble());
    // DogLog.log("EndEffector/endEffectorOuterCANrangeDistance", endEffectorOuterCANrange.getDistance().getValueAsDouble());
    // DogLog.log("EndEffector/endEffectorOuterCANrangeBoolean", endEffectorOuterCANrange.getIsDetected().getValue());

    SmartDashboard.putNumber("End Effector rotations", m_endEffectorPivot.getPosition().getValueAsDouble());


  }
}
