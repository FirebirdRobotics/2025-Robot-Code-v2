// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeUntilendEffectorOuterCANrange extends Command {
  /** Creates a new runIntakeRollersUntillIntakeCANRange. */
  Intake m_Intake;
  EndEffector m_EndEffector;

  public intakeUntilendEffectorOuterCANrange(Intake intake, EndEffector endEffector) {
    m_Intake = intake;
    m_EndEffector = endEffector;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake, m_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_Intake.setAllClosedLoopConfigsToCorrectValues();
    m_Intake.setRollerMotorPercentOutput(0.6);
    m_Intake.goToDeployedPosition();
    m_EndEffector.setRollerMotorPercentOutput(-0.2);
    // if (m_Intake.getIntakePivotAngle() > 5.0) {
    //   m_Intake.setAllClosedLoopConfigsTo0();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Intake.setAllClosedLoopConfigsToCorrectValues();
    m_Intake.setRollerMotorPercentOutput(0);
    m_Intake.goToFramePerimeterPosition();
    m_EndEffector.setRollerMotorPercentOutput(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffector.getendEffectorOuterCANrange();
    
  }
}
