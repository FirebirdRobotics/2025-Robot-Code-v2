/*Credit to Metal Crusaders for code: https://github.com/Metal-Crusaders/Reefscape2025Code/tree/main/src/main/java/frc/robot/commands/swerve */
package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPose extends Command {

    private final CommandSwerveDrivetrain swerve;
    private Pose2d poseFinal, currentPose;

    private final PIDController xTranslationPID, yTranslationPID;
    private final PIDController rotationPID;

    public CloseDriveToPose(CommandSwerveDrivetrain swerve, Pose2d finalPose) {
        this.swerve = swerve;
        this.poseFinal = finalPose;
        this.xTranslationPID = new PIDController(5.0, 
                                                0, 
                                                0);
        this.yTranslationPID = new PIDController(5.0, 
                                                0, 
                                                0);
        this.rotationPID = new PIDController(5.0, 
                                             0, 
                                             0);

        this.rotationPID.enableContinuousInput(-1 * Math.PI, Math.PI);
        
        xTranslationPID.setTolerance(0.1);
        yTranslationPID.setTolerance(0.1);
        rotationPID.setTolerance(0.01);
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentPose = swerve.getState().Pose;

        xTranslationPID.reset();
        yTranslationPID.reset();
        rotationPID.reset();

        xTranslationPID.setSetpoint(poseFinal.getX());
        yTranslationPID.setSetpoint(poseFinal.getY());
        rotationPID.setSetpoint(poseFinal.getRotation().getRadians());
    }

    @Override
    public void execute() {
        currentPose = swerve.getState().Pose;
        
        double xSpeed = xTranslationPID.calculate(currentPose.getX());
        double ySpeed = yTranslationPID.calculate(currentPose.getY());
        double thetaSpeed = rotationPID.calculate(currentPose.getRotation().getRadians());
        
        ChassisSpeeds wheelSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        swerve.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(wheelSpeeds));
    }

    @Override
    public boolean isFinished() {
        boolean xTranslationDone = xTranslationPID.atSetpoint();
        boolean yTranslationDone = yTranslationPID.atSetpoint();
        boolean rotationDone = rotationPID.atSetpoint();
        
        return xTranslationDone && yTranslationDone && rotationDone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}