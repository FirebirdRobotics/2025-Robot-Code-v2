// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.subsystems.EndEffector;
// import frc.robot.subsystems.Intake;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class IntakingWithIntakeUpWhileCoralInBot extends ParallelDeadlineGroup {
//   EndEffector m_EndEffector;
//   Intake m_Intake;

//   deployIntakeUntilIntakeCANrange m_DeployIntakeUntilIntakeCANrange;
//   RunEndEffectorUntilEndEffectorCANrange m_RunEndEffectorRollerUntilEndEffectorCANrange = new RunEndEffectorUntilEndEffectorCANrange(m_EndEffector);

//   /** Creates a new IntakingWithIntakeUpWhileCoralInBot. */
//   public IntakingWithIntakeUpWhileCoralInBot(Intake intake, EndEffector endEffector, RunEndEffectorUntilEndEffectorCANrange runEndEffectorUntilEndEffectorCANrange) {
    
//     super(m_RunEndEffectorRollerUntilEndEffectorCANrange);

//     m_Intake = intake;
//     m_EndEffector = endEffector;
//     m_DeployIntakeUntilIntakeCANrange = new deployIntakeUntilIntakeCANrange(m_Intake);

    
//     // Add the deadline command in the super() call. Add other commands using
//     // addCommands().
//     addCommands(m_DeployIntakeUntilIntakeCANrange);
//     // addCommands(new FooCommand(), new BarCommand());
//   }
// }
