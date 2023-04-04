package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants;
import frc.robot.MathMethods;

public class OTFTrajectoryCmd extends CommandBase {

    private final DriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final List<Translation2d> listOfTranslations;
    private final Pose2d endPose;
    private final TrajectoryConfig config;

    public OTFTrajectoryCmd(DriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, List<Translation2d> listOfTranslations, Pose2d endPose, TrajectoryConfig config) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.listOfTranslations = listOfTranslations;
        this.endPose = endPose;
        this.config = config;
        this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
        addRequirements(swerveSubsystem);
    }
    

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory path = TrajectoryGenerator.generateTrajectory(swerveSubsystem.getPose(), listOfTranslations, endPose, config);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            path,
            swerveSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);
        
        new SequentialCommandGroup(swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules()));
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}