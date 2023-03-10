package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;




public class Trajectories {
    public static final TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kAutoMaxSpeedMetersPerSecond,
                AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    
    public static final TrajectoryConfig reverseConfig = config.setReversed(true);

    public static final TrajectoryConfig lameConfig =
        new TrajectoryConfig(
                AutoConstants.kLameSpeedCap,
                AutoConstants.kLameAccelCap)
            .setKinematics(DriveConstants.kDriveKinematics);


    public static final Trajectory defaultTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    public static final Trajectory chargeLeftBlueTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.70, 4.38, new Rotation2d(Math.PI/2)),
            List.of(new Translation2d(5.56, 4.60)),
            new Pose2d(5.22, 2.95, new Rotation2d(Math.PI/2)),
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

    
    //One Cargo and Balance Auto - drive to charge plate
    public static final Trajectory blueTopToCharge = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(4.2, 4.8), new Translation2d(6.8, 4.6)),
            new Pose2d(5.2, 3.2, new Rotation2d(-Math.PI/2)), 
            reverseConfig);

    //One Cargo and Balance Auto - drive up charge plate
    public static final Trajectory blueTopToChargeUp = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.2, 3.2, new Rotation2d(-Math.PI/2)), 
            List.of(),
            new Pose2d(4.7, 3.2, new Rotation2d(-Math.PI/2)), 
            lameConfig);

    //Double cargo autonomous - drive to cargo
    public static final Trajectory blueTopTo2Cargo = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(4.2, 4.8)),
            new Pose2d(6.7, 4.6, new Rotation2d(0)), 
            reverseConfig);

    //Theoretically Impossible Without negligible air resistance and/or friction  /j
    //Double cargo autonomous - drive back to cube node from cargo pickup
    public static final Trajectory blueTopReturnCargo = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(6.7,4.6, new Rotation2d(0)), 
            List.of(new Translation2d(4.2, 4.8)), 
            new Pose2d(1.7, 4.45, new Rotation2d(-Math.PI)), 
            reverseConfig);


}
