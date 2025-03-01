// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestIntake extends SubsystemBase {

    private final TalonFX m_rollerMotor = new TalonFX(15, "roborio");

  /** Creates a new TestIntake. */
  public TestIntake() {
    // var talonFXConfigs = new TalonFXConfiguration();

    // // set slot 0 gains
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0; // no output for error derivative
    
    // // set Motion Magic Velocity settings
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    // motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
    // m_rollerMotor.getConfigurator().apply(talonFXConfigs);
    
  }

  public void runIntake(double velocity) {
      final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
      m_request.Acceleration = 100; // rot/s^2

      m_rollerMotor.setControl(m_request.withVelocity(velocity));

  }

    public Command setIntakePower(double power) {
    
    return runEnd(
      () -> runIntake(20),
      () -> runIntake(0)
    ); 

  }

  public void runIntakeVoltage(double power) {
    m_rollerMotor.set(power);
  }

  public Command setIntakePowerCommand(double power) {
    
    return runEnd(
      () -> runIntakeVoltage(power),
      () -> runIntakeVoltage(power)
    ); 

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
