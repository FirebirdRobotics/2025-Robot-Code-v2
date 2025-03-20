// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AlignToReef;
// import frc.robot.Commands.IntakingWithIntakeUpWhileCoralInBot;
import frc.robot.Commands.intakeUntilendEffectorOuterCANrange;
import frc.robot.Commands.intakeUntillIntakeCANRange;
import frc.robot.Commands.juggleCoralTillRight;
import frc.robot.Commands.runEndEffectorUntilEndEffectorOuterCANrange;
import frc.robot.Commands.AlignToReef.ReefSide;
import frc.robot.Commands.runEndEffectorUntilEndEffectorOuterCANrange;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.TestIntake;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    // TestIntake m_TestIntake = new TestIntake();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    CommandGenericHID  buttonBoardRight = new CommandGenericHID(2);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    Elevator m_Elevator = new Elevator();

    EndEffector m_EndEffector = new EndEffector();

    Intake intake = new Intake();

    Vision vision;

    LEDs m_Leds = new LEDs();

    Command m_RunIntakeRollersUntillIntakeCANRange = new intakeUntillIntakeCANRange(intake);

    Command m_intakeUntilendEffectorOuterCANrange = new intakeUntilendEffectorOuterCANrange(intake, m_EndEffector);

    runEndEffectorUntilEndEffectorOuterCANrange m_runEndEffectorUntilendEffectorOuterCANrange = new runEndEffectorUntilEndEffectorOuterCANrange(m_EndEffector);


    juggleCoralTillRight m_juggleCoralTillRight = new juggleCoralTillRight(m_EndEffector);

    intakeUntillIntakeCANRange m_intakeUntillIntakeCANRange = new intakeUntillIntakeCANRange(intake);
    // Command m_IntakingWithIntakeUpWhileCoralInBot = new IntakingWithIntakeUpWhileCoralInBot(intake, m_EndEffector, m_runEndEffectorUntilendEffectorOuterCANrange);
    
    

    

    public RobotContainer() {

        NamedCommands.registerCommand("L1 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL1(),m_EndEffector.goToL1()));
        NamedCommands.registerCommand("L3 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL3(),m_EndEffector.goToL3()));

        NamedCommands.registerCommand("L4 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL4(),m_EndEffector.goToL4()));

        NamedCommands.registerCommand("Outake Coral", m_EndEffector.setRollerMotorPercentOutputAndThenTo0Command( 0.35).withTimeout(3));

        FollowPathCommand.warmupCommand().schedule();

        
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withNtPublish(true));
        DogLog.setPdh(new PowerDistribution());

        DogLog.log("ExampleLog", "Hello world!");

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        vision = new Vision(drivetrain);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

/*MAKE SURE TO UNCOMMENT ABOVE SO IT CAN DRIVE!!!! */

        // joystick.rightBumper().onTrue(drivetrain.goToCorralStationA());
        // joystick.rightBumper().onTrue(drivetrain.pathfindToReefA());

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.a().whileTrue(intake.CommandGoToAngle(5));

        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper().whileTrue(new AlignToReef(drivetrain, ReefSide.LEFT, false));
        // joystick.leftBumper().whileTrue(intake.testIntakeDeployAndUndeploy());
        
        // joystick.leftBumper().whileTrue(Commands.sequence(m_intakeUntilendEffectorOuterCANrange));

        // joystick.a().whileTrue(intake.setRollerMotorPercentOutputCommand(0.6));
        // joystick.rightBumper().whileTrue(Commands.sequence(intake.setRollerMotorPercentOutputAndThenTo0Command(-0.15), intake.goToFramePerimeterPositionCommand()));

        // buttonBoardRight.button(1).whileTrue(m_Elevator.goToL1());
        buttonBoardRight.button(1).whileTrue(Commands.parallel(m_Elevator.goToL1(),m_EndEffector.goToL1()));

        buttonBoardRight.button(2).whileTrue(Commands.parallel(m_Elevator.goToL2(),m_EndEffector.goToL2()));
        buttonBoardRight.button(3).whileTrue(Commands.parallel(m_Elevator.goToL3(),m_EndEffector.goToL3()));
        buttonBoardRight.button(4).whileTrue(Commands.parallel(m_Elevator.goToL4(),m_EndEffector.goToL4()));

        buttonBoardRight.button(5).whileTrue(Commands.parallel(m_Elevator.goToStowedPosition(),m_EndEffector.goToStowed()));
        buttonBoardRight.button(6).whileTrue((m_EndEffector.setRollerMotorPercentOutputAndThenTo0Command(0.3)));
        buttonBoardRight.button(7).whileTrue(Commands.sequence(intake.setRollerMotorPercentOutputAndThenTo0Command(-0.25), intake.goToFramePerimeterPositionCommand()));
        // buttonBoardRight.button(8).whileTrue((m_juggleCoralTillRight)); // THIS Crashes the code!
        buttonBoardRight.button(9).whileTrue(
            Commands.parallel(intake.goToDeployAndThenToUndeployCommand(), m_juggleCoralTillRight));;





        // joystick.leftBumper().whileTrue(m_TestIntake.setIntakePowerCommand(-0.7));
        
    }

    /*
     * 
 intake drops to proper angle -- Roller motor rotating rollers at specified voltage -- End effector motors rotating wheels at voltage
        	|					                    |				                                     |
        	|					                    |----------------------------------------------------|
        Ends When Intake 	                            Ends When End effector CANRange detects coral
    CANRange Detects Coral
     * 
     * 
     * 
     */

    public Command manualIntakeCoral() {
        return null;
    }

    
    public Command autonomousIntakeCoral() {
        return null;
    }







    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        // return Commands.sequence(intake.goToFramePerimeterPositionCommand(),drivetrain.driveOutSimpleCommand());
    }
}
