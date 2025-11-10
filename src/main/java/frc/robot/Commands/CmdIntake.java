// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class CmdIntake extends Command  {
  /** Creates a new runIntakeRollersUntillIntakeCANRange. */
  Intake m_Intake;

  public CmdIntake(Intake intake) {
    m_Intake = intake;
    addRequirements(m_Intake);
  }

  @Override
  public void execute() {
    // Continuously run while the command is active
    m_Intake.goToDeployedPosition();
  }

  @Override
  public void end(boolean interrupted) {
    // Stops everything when the command ends
    m_Intake.goToFramePerimeterPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
