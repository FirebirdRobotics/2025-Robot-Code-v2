// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class juggleCoralTillRight extends Command {
  EndEffector m_EndEffector;
  LEDs m_LEDs;

  /** Creates a new juggleCoralRillRight. */
  public juggleCoralTillRight(EndEffector endEffector, LEDs leds) {
    m_EndEffector = endEffector;
    m_LEDs = leds;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_EndEffector.getendEffectorInnerCANrange() && (m_EndEffector.getendEffectorOuterCANrange() == false)) {
      m_EndEffector.setRollerMotorPercentOutput(0.15);
    }

    if ((m_EndEffector.getendEffectorInnerCANrange() == false) && (m_EndEffector.getendEffectorOuterCANrange())) {
      m_EndEffector.setRollerMotorPercentOutput(-0.15);
    }

    if ((m_EndEffector.getendEffectorInnerCANrange() == false) && (m_EndEffector.getendEffectorOuterCANrange() == false)) {
      m_EndEffector.setRollerMotorPercentOutput(0.15);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffector.setRollerMotorPercentOutput(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_EndEffector.getendEffectorInnerCANrange() && m_EndEffector.getendEffectorOuterCANrange()) {
      return true;
    }
    return false;
  }
}
