package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.DriveSubsystem;



public class Trajectories {
    public static final TrajectoryConfig config =
    new TrajectoryConfig(
            AutoConstants.kAutoMaxSpeedMetersPerSecond,
            AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    public static final Trajectory defaultTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    public static final Trajectory chargeLeftBlueTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.70, 4.38, new Rotation2d(0)),
        List.of(new Translation2d(5.56, 4.60)),
        new Pose2d(5.22, 2.92, new Rotation2d(Math.PI)),
        config);
        
    public static final Trajectory chargeRightBlueTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.70, 1.08, new Rotation2d(0)),
        List.of(new Translation2d(3.91, 0.62), new Translation2d(5.92, 0.94)),
        new Pose2d(5.21, 2.12, new Rotation2d(Math.PI)),
        config);

    public static final Trajectory chargeLeftRedTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.70, 4.38, new Rotation2d(0)),
        List.of(new Translation2d(AutoConstants.kFieldEndXCoordinate - 5.56, 4.60)),
        new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.22, 2.92, new Rotation2d(Math.PI)),
        config);

    public static final Trajectory chargeRightRedTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.70, 1.08, new Rotation2d(0)),
        List.of(new Translation2d(AutoConstants.kFieldEndXCoordinate - 3.91, 0.62), new Translation2d(AutoConstants.kFieldEndXCoordinate - 5.92, 0.94)),
        new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.21, 2.12, new Rotation2d(Math.PI)),
        config);

    public static final Trajectory blueTopToCargo = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.7, 5, new Rotation2d(-Math.PI)), 
        List.of(new Translation2d(4.2, 4.8)),
        new Pose2d(6.65,4.6, new Rotation2d(0)), 
        config);

    public static final Trajectory blueTopReturnCargo = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.65,4.6, new Rotation2d(0)), 
        List.of(new Translation2d(4.2, 4.8)), 
        new Pose2d(1.7, 5, new Rotation2d(-Math.PI)), 
        config);


}
