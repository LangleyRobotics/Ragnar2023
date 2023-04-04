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
                1,
                1)
            .setKinematics(DriveConstants.kDriveKinematics);

    public static final TrajectoryConfig reverseLameConfig = lameConfig.setReversed(true);


    public static final Trajectory defaultTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    //BLUE LEFT
    //One Cargo and Balance Auto - drive to charge plate
    public static final Trajectory blueTopToChargeLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(4.2, 4.8), new Translation2d(6.8, 4.6)),
            new Pose2d(5.7, 3.2, new Rotation2d(0)), 
            reverseConfig);

    //One Cargo and Balance Auto - drive up charge plate
    public static final Trajectory blueTopToChargeUpLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.7, 3.2, new Rotation2d(0)), 
            List.of(),
            new Pose2d(2.50, 3.2, new Rotation2d(0)), 
            reverseConfig);

    //BLUE RIGHT
    //One Cargo and Balance Auto - drive to charge plate
    public static final Trajectory blueTopToChargeRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 1.06, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(3.25, 0.62), new Translation2d(5.8, 0.97)),
            new Pose2d(5.8, 2.35, new Rotation2d(0)), 
            reverseConfig);

    //One Cargo and Balance Auto - drive up charge plate
    public static final Trajectory blueTopToChargeUpRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.8, 2.35, new Rotation2d(0)), 
            List.of(),
            new Pose2d(2.60, 2.35, new Rotation2d(0)), 
            reverseConfig);


    //RED RIGHT
    //One Cargo and Balance Auto - drive to charge plate
    public static final Trajectory redTopToChargeRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.7, 4.45, new Rotation2d(0)), 
            List.of(new Translation2d(AutoConstants.kFieldEndXCoordinate - 4.2, 4.8), new Translation2d(AutoConstants.kFieldEndXCoordinate - 6.8, 4.6)),
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.7, 3.2, new Rotation2d(Math.PI)), 
            reverseConfig);

    //One Cargo and Balance Auto - drive up charge plate
    public static final Trajectory redTopToChargeUpRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.7, 3.2, new Rotation2d(Math.PI)), 
            List.of(),
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 2.50, 3.2, new Rotation2d(Math.PI)), 
            reverseConfig);

    //RED LEFT
    //One Cargo and Balance Auto - drive to charge plate
    public static final Trajectory redTopToChargeLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.7, 1.06, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(AutoConstants.kFieldEndXCoordinate - 3.25, 0.62), new Translation2d(AutoConstants.kFieldEndXCoordinate - 5.8, 0.97)),
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.8, 2.35, new Rotation2d(0)), 
            reverseConfig);

    //One Cargo and Balance Auto - drive up charge plate
    public static final Trajectory redTopToChargeUpLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.8, 2.35, new Rotation2d(0)), 
            List.of(),
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 2.60, 2.35, new Rotation2d(0)), 
            reverseConfig);






    //Double cargo autonomous - drive to cargo
    public static final Trajectory blueTopTo2Cargo = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.toRadians(180.0))), 
            List.of(new Translation2d(4.2, 4.8)),
            new Pose2d(5.4, 4.6, new Rotation2d(Math.toRadians(-3.0))), 
            reverseConfig);

    public static final Pose2d blueDoubleScoreResetPose = new Pose2d(5.3, 4.6, new Rotation2d(Math.toRadians(180.0)));


    //Theoretically Impossible Without negligible air resistance and/or friction  /j
    //Double cargo autonomous - drive back to cube node from cargo pickup
    public static final Trajectory blueTopReturnCargo = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.3,4.6, new Rotation2d(Math.PI)), 
            List.of(), 
            new Pose2d(2.53, 4.6, new Rotation2d(Math.PI)), 
            reverseConfig);

     public static final Trajectory redTopReturnCargo = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(4.2, 4.8)),
            new Pose2d(6.7, 4.6, new Rotation2d(0)), 
            reverseConfig);
    

    //Backup Auton
    public static final Trajectory backAutoRedRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.7, 4.45, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.7,4.45, new Rotation2d(0)), 
            reverseConfig);
    
    
    public static final Trajectory backAutoBlueLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.7, 4.45, new Rotation2d(Math.PI)), 
            List.of(), 
            new Pose2d(5.7,4.45, new Rotation2d(Math.PI)), 
            reverseConfig);
    
    public static final Trajectory backAutoRedLeft = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(AutoConstants.kFieldEndXCoordinate - 1.7, 1.06, new Rotation2d(0)), 
                List.of(), 
                new Pose2d(AutoConstants.kFieldEndXCoordinate - 5.95,1.06, new Rotation2d(0)), 
                reverseConfig);
        
        
    public static final Trajectory backAutoBlueRight = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.7, 1.06, new Rotation2d(Math.PI)), 
                List.of(), 
                new Pose2d(5.95,1.06, new Rotation2d(Math.PI)), 
                reverseConfig);


    public static final Trajectory backToBalTest = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1,0, new Rotation2d(Math.PI)), 
            List.of(), 
            new Pose2d(0.55, 0, new Rotation2d(Math.PI)), 
            reverseConfig);


    public static final Trajectory testDouble = 
        TrajectoryGenerator.generateTrajectory(new Pose2d(6.5,4.5, 
            new Rotation2d(Math.toRadians(-22))),
            List.of(new Translation2d(5.3, 4.6)), 
            new Pose2d(2.53, 4.5, new Rotation2d(Math.toRadians(181))), 
            reverseConfig);


    public static final Trajectory dynamicTrajectoryTestP1 = 
    TrajectoryGenerator.generateTrajectory(new Pose2d(1.0, -1.0, 
        new Rotation2d(Math.toRadians(180.0))),
        List.of(), 
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))), 
        config);

}
