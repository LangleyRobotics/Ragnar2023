// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

//Constants Imports 
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Buttons;
import frc.robot.commands.AllForNaught;
import frc.robot.Trajectories;
//Command Imports
import frc.robot.commands.ClawCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LiftAutoCmd;
import frc.robot.commands.LiftDumbSpeed;
import frc.robot.commands.LiftIntakeCmd;
import frc.robot.commands.LiftTriggerCmd;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.TelescopeCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SullyCmd;

//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.time.Instant;
import java.util.List;

import javax.print.attribute.TextSyntax;
import javax.xml.crypto.dsig.Manifest;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final LiftSubsystem robotLift = new LiftSubsystem();
  private final ManipulatorSubsystem robotManipulator = new ManipulatorSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);

  //COMMENT OUT THE CONTROLLER BELOW - ONLY USE WHEN NEED PS4 CONTROLLER
  //PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  PathConstraints standardAutoPathConstraints = new PathConstraints(AutoConstants.kAutoMaxSpeedMetersPerSecond, AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared);
  
  /*TrajectoryConfig config =
  new TrajectoryConfig(
          AutoConstants.kAutoMaxSpeedMetersPerSecond,
          AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);
  
  Trajectory defaultTrajectory =
  TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(2, 0, new Rotation2d(0)),
      config);*/
  
//PP is for noobs
/* 
  //PathPlannerTrajectory defaultTrajectory = PathPlanner.loadPath("Default", standardAutoPathConstraints);
  PathPlannerTrajectory chargePlateLeftTrajectory = PathPlanner.loadPath("Charge Plate Run-up Left", standardAutoPathConstraints);
  PathPlannerTrajectory blueTopScoreTrajectory = PathPlanner.loadPath("Scoring Path Blue Top", standardAutoPathConstraints);
  PathPlannerTrajectory chargeRightTrajectory = PathPlanner.loadPath("Charge Plate Run-up Right", standardAutoPathConstraints);
*/
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure default commands
    robotDrive.setDefaultCommand(
        new SwerveControllerCmd(
            robotDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true));
    
    robotLift.setDefaultCommand(
        new LiftTriggerCmd(
          robotLift,
          () -> operatorController.getLeftTriggerAxis(),
          () -> operatorController.getRightTriggerAxis()));

    robotManipulator.setDefaultCommand(new IntakeCmd(robotManipulator, () -> 0.0, 0));


    
    SequentialCommandGroup defaultPathCmd = robotDrive.AutoCommandFactory(Trajectories.defaultTrajectory);
    SequentialCommandGroup chargeLeftBlueCmd = robotDrive.AutoCommandFactory(Trajectories.chargeLeftBlueTrajectory);
    SequentialCommandGroup chargeRightBlueCmd = robotDrive.AutoCommandFactory(Trajectories.chargeRightBlueTrajectory);
    SequentialCommandGroup chargeLeftRedCmd = robotDrive.AutoCommandFactory(Trajectories.chargeLeftRedTrajectory);
    SequentialCommandGroup chargeRightRedCmd = robotDrive.AutoCommandFactory(Trajectories.chargeRightRedTrajectory);

    SequentialCommandGroup blueTopToCargoCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopToCargo);
    SequentialCommandGroup blueTopCargoReturnCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopReturnCargo);
    
    

    LiftAutoCmd liftAuto = new LiftAutoCmd(robotLift, LiftConstants.kMaxLiftPosition);
    LiftDumbSpeed liftMaintainCmd = new LiftDumbSpeed(robotLift, 0.5);
    LiftIntakeCmd liftIntake = new LiftIntakeCmd(robotLift);
    SullyCmd toggleSully = new SullyCmd(pneumaticsSubsystem);
    TelescopeCmd telescopeAutoExtend = new TelescopeCmd(robotLift, () -> 1.00, () -> -1);
    TelescopeCmd telescopeAutoRetract = new TelescopeCmd(robotLift, () -> 1.00, () -> 1);
    TelescopeCmd telescopeAutoStop = new TelescopeCmd(robotLift, () -> 0.0, () -> 1);

    ClawCmd autoOuttake = new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1);
    WaitCommand sullyLiftDelay = new WaitCommand(2.0);
    WaitCommand telescopeTime = new WaitCommand(3.5);
    WaitCommand outtakeTime = new WaitCommand(1.5);
    //Sequential Command Groups
    SequentialCommandGroup sullyThenLift = new SequentialCommandGroup(toggleSully, sullyLiftDelay, liftAuto);
    SequentialCommandGroup Test1 = new SequentialCommandGroup(toggleSully, sullyLiftDelay, liftAuto, telescopeTime);
    SequentialCommandGroup Test2 = new SequentialCommandGroup(toggleSully, sullyLiftDelay, liftAuto, telescopeAutoExtend.withTimeout(3.5));
    /* 
    SequentialCommandGroup preloadScore = new SequentialCommandGroup(toggleSully, sullyLiftDelay, liftAuto, liftMaintainCmd, telescopeAutoExtend.withTimeout(3.5)), 
    telescopeAutoStop, autoOuttake.withTimeout(0.5), telescopeAutoRetract);
    */
    


    autoChooser.setDefaultOption("Default Auto", defaultPathCmd);
    autoChooser.addOption("Charge Left Blue Auto", chargeLeftBlueCmd);
    autoChooser.addOption("Charge Right Blue Auto", chargeRightBlueCmd);
    autoChooser.addOption("Charge Left Red Auto", chargeLeftRedCmd);
    autoChooser.addOption("Charge Right Red Auto", chargeRightRedCmd);
    autoChooser.addOption("Lift Arm", sullyThenLift);
    autoChooser.addOption("Test 1`", Test1);
    autoChooser.addOption("Test 2", Test2);
    autoChooser.addOption("Blue Top To Cargo", blueTopToCargoCmd);
    autoChooser.addOption("Blue top to cargo and back.", blueTopToCargoCmd.andThen(telescopeTime, blueTopCargoReturnCmd));
    autoChooser.addOption("Lift Maintain Test", liftMaintainCmd);
    //autoChooser.addOption("Full Auto Test", fullAutoTest);

    //autoChooser.addOption("High Score to Balance", highScoreAuto_Balance);

    SmartDashboard.putData(autoChooser);


    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(operatorController, Buttons.X).whileTrue(new IntakeCmd(robotManipulator, 
      () -> ManipulatorConstants.kIntakeMotorSpeed, 1));
    new JoystickButton(operatorController, Buttons.B).whileTrue(new IntakeCmd(robotManipulator, 
      () -> ManipulatorConstants.kIntakeMotorSpeed, -1));
    new JoystickButton(operatorController, Buttons.Y).whileTrue(new ClawCmd(robotManipulator, 
      () -> ManipulatorConstants.kClawMotorSpeed, () -> 1));
    
    new JoystickButton(operatorController, Buttons.A).whileTrue(new ClawCmd(robotManipulator, 
      () -> ManipulatorConstants.kClawMotorSpeed, () -> -1));

    new JoystickButton(driverController, Buttons.LB).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    new JoystickButton(driverController, Buttons.RB).whileTrue(new RumbleCmd(operatorController, 2, 1.00));

    
    new POVButton(operatorController, Buttons.LEFT_ARR).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new RumbleCmd(driverController, 2, 1.00));
    
    new POVButton(operatorController, Buttons.UP_ARR).toggleOnTrue(new SullyCmd(pneumaticsSubsystem));
    new JoystickButton(operatorController, Buttons.LB).whileTrue(new TelescopeCmd(robotLift, ()-> LiftConstants.kTelescopeSpeed, () -> 1));
    new JoystickButton(operatorController, Buttons.RB).whileTrue(new TelescopeCmd(robotLift, ()-> LiftConstants.kTelescopeSpeed, () -> -1));
    
    
    new JoystickButton(operatorController, Buttons.Maria).toggleOnTrue(new LiftIntakeCmd(robotLift));

    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));



    //Balance
    //IMPLEMENT ACCEL CAP FROM MATH METHODS


      new JoystickButton(driverController, Buttons.A).whileTrue(new SwerveControllerCmd(robotDrive, 
        () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond)),
        () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond)),
        () -> (0.0), () -> false));

      SmartDashboard.putNumber("Auto Balance X Speed", MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond));
      SmartDashboard.putNumber("Auto Balance Y Speed", MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    /* 
    int autoNum = 1;
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kAutoMaxSpeedMetersPerSecond,
                AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(/*new Translation2d(1, 1), new Translation2d(2, -1)*//* ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    //TO-DO
    
    - Create turn method to seek AprilTags 
    - Detect if multiple AprilTags are in view???

     */ /* 
    PathPlannerTrajectory defaultTrajectory = PathPlanner.loadPath("Default", 1.0, 1.0);

    ProfiledPIDController thetaController =
    new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand testNoPP = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    SwerveControllerCommand testPP = new SwerveControllerCommand(
      defaultTrajectory,
      robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      robotDrive::setModuleStates,
      robotDrive);

    SequentialCommandGroup SequentialCommandOutputNoPP = new SequentialCommandGroup(new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                                                                                    testNoPP,
                                                                                    new InstantCommand(() -> robotDrive.stopModules()));

     SequentialCommandGroup SequentialCommandOutputPP = new SequentialCommandGroup(new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                                                                                    testPP,
                                                                                    new InstantCommand(() -> robotDrive.stopModules()));
    
  


    
    switch(autoNum) {
      case 0:
        return SequentialCommandOutputNoPP;
      case 1:
        return SequentialCommandOutputPP;
      default:
        return SequentialCommandOutputNoPP;
    }
    */
    

  }
}
