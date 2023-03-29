// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import com.pathplanner.lib.PathConstraints;


//Constants Imports 
import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AllForNaught;
//import frc.robot.Trajectories;
import frc.robot.commands.AprilReorientation;
//Command Imports
import frc.robot.commands.ClawCmd;
import frc.robot.commands.DropDownExclusiveCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.AutoCommands.LiftAutoCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SallyCmd;
import frc.robot.commands.SullyCmd;
//import frc.robot.commands.LiftAutoCmd;
import frc.robot.commands.LiftIntakeCmd;
import frc.robot.commands.LiftTriggerCmd;
import frc.robot.commands.LiftTriggerCmdBounded;
//import frc.robot.commands.ResetLiftBoundsCmd;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.TelescopeCmd;
import frc.robot.commands.TelescopeWithLiftBoundless;
import frc.robot.commands.TelescopeWithLiftCmd;
import frc.robot.commands.AutoCommands.RetractManipSeqCmdGrp;
import frc.robot.commands.AutoCommands.outtakeAutoSeqCmdGrp;
import frc.robot.commands.lightingCommands.BiPrideCmd;
import frc.robot.commands.lightingCommands.TransPrideCmd;
import frc.robot.commands.lightingCommands.rainbow;
import frc.robot.commands.lightingCommands.rainbowFromMid;


//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.lightingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;



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

  private final lightingSubsystem lightingSubsystem = new lightingSubsystem();



  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);

  //COMMENT OUT THE CONTROLLER BELOW - ONLY USE WHEN NEED PS4 CONTROLLER
  //PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  //SendableChooser<Command> lightingChooser = new SendableChooser<>();
  
  PathConstraints standardAutoPathConstraints = new PathConstraints(AutoConstants.kAutoMaxSpeedMetersPerSecond, AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared);
  
//PathPlanner is for noobs
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

    

    //lightingSubsystem.setDefaultCommand(new CargoSignalCmd(lightingSubsystem, pneumaticsSubsystem));
    lightingSubsystem.setDefaultCommand(new rainbowFromMid(lightingSubsystem));


    //Load in paths from Trajectories as drive commands using the AutoCommandFactory
    SequentialCommandGroup S_Curve = robotDrive.AutoCommandFactory(Trajectories.defaultTrajectory);
    

    //Create telescope and lift commands
    ParallelRaceGroup teleAutoRetractWithArmResist = new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2);
    ParallelRaceGroup armDownTele = new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5);


    //Create command sequence that stows telescope, lift, and drop-down intake
    SequentialCommandGroup retractManipulators = new SequentialCommandGroup(teleAutoRetractWithArmResist, armDownTele, new SullyCmd(pneumaticsSubsystem));
    SequentialCommandGroup retractManipulators2 = new SequentialCommandGroup(
      new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> 0.90, () -> 0.0, false).withTimeout(2.5));
    SequentialCommandGroup retractManipulators3 = new SequentialCommandGroup(
      new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
      new SullyCmd(pneumaticsSubsystem));

    //Create parallel commands to perform manipulator stow while driving
    ParallelCommandGroup driveBackRetractBlueLeft = new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeLeft));
    
    ParallelCommandGroup toCargo2AndRetractBlueLeft = new ParallelCommandGroup(new SequentialCommandGroup(
      new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new LiftAutoCmd(robotLift, LiftConstants.kIntakeLiftPosition)), robotDrive.AutoCommandFactory(Trajectories.blueTopTo2Cargo));

    ParallelCommandGroup driveBackRetractRedRight = new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.redTopToChargeRight));


    //21 pt autonomous - Score preloaded cargo, drive around the community boundary to the charge plate, balance on charge plate
    SequentialCommandGroup fullTwentyOnePtAutonomousBlueLeft = new SequentialCommandGroup(
      outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem), 
      new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), 
      new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeLeft)))).andThen(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUpLeft), 
      new SwerveControllerCmd(robotDrive, true));

    SequentialCommandGroup fullTwentyOnePtAutonomousBlueRight = new SequentialCommandGroup(
      outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), 
      new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeRight)))).andThen(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUpRight), 
      new SwerveControllerCmd(robotDrive, true));

    SequentialCommandGroup fullTwentyOnePtAutonomousRedRight = new SequentialCommandGroup(
      outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem), 
      new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), 
      new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.redTopToChargeRight)))).andThen(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUpRight), 
      new SwerveControllerCmd(robotDrive, true));

    SequentialCommandGroup fullTwentyOnePtAutonomousRedLeft = new SequentialCommandGroup(
      outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      new ParallelCommandGroup(RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), 
      new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.redTopToChargeLeft)))).andThen(robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUpRight), 
      new SwerveControllerCmd(robotDrive, true));

    
    

    ParallelCommandGroup captureCargo = new ParallelCommandGroup(new SwerveControllerCmd(robotDrive, () -> (0.15), () -> (0.0), 
    () -> (-MathMethods.speedMax2(0.04*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
    () -> false).withTimeout(1.5), 
    new LiftIntakeCmd(robotLift));

    //SequentialCommandGroup prepareToPounceTest = new SequentialCommandGroup(new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), prepareToPounce);

    ParallelRaceGroup captureCargo2 = new ParallelRaceGroup(new SwerveControllerCmd(robotDrive, () -> 0.4, () -> 0.0, () -> 0.0, () -> true),
     new IntakeCmd(robotManipulator, () -> ManipulatorConstants.kIntakeMotorSpeed, -1), new WaitCommand(1.0));

    //Unfinished 15 pt autonomous - Score preloaded cargo, drive out of commmunity boundary to pick up cargo, drive back to cube node, score new cargo
    
    SequentialCommandGroup doubleScoringAuto = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem), 
        toCargo2AndRetractBlueLeft,
        captureCargo
        /*robotDrive.AutoCommandFactory(TrajectoryGenerator.generateTrajectory(
            MathMethods.doubleArrToPose2d(LimelightSubsystem.get_LL_botpose()),
            List.of(new Translation2d(4.2, 4.8)),
            new Pose2d(2.4, 4.45, new Rotation2d(Math.toRadians(180.0))),
            Trajectories.config)),*/
        /*outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem)*/);


      SequentialCommandGroup singleScoreAuto = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem));

      SequentialCommandGroup singleScoreBlueThenBackLeft = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.backAutoBlueLeft));
      
      SequentialCommandGroup singleScoreRedThenBackLeft = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.backAutoRedLeft));

      SequentialCommandGroup singleScoreBlueThenBackRight= new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.backAutoBlueRight));
      
      SequentialCommandGroup singleScoreRedThenBackRight = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      RetractManipSeqCmdGrp.getSequentialCommandGroup(robotLift, pneumaticsSubsystem), robotDrive.AutoCommandFactory(Trajectories.backAutoRedRight));


    SequentialCommandGroup driveBackAutoRedLeft = robotDrive.AutoCommandFactory(Trajectories.backAutoRedLeft);
    SequentialCommandGroup driveBackAutoBlueLeft = robotDrive.AutoCommandFactory(Trajectories.backAutoBlueLeft);
    SequentialCommandGroup driveBackAutoRedRight = robotDrive.AutoCommandFactory(Trajectories.backAutoRedRight);
    SequentialCommandGroup driveBackAutoBlueRight = robotDrive.AutoCommandFactory(Trajectories.backAutoBlueRight);

    //Add autonomous commands to drop-down selector on ShuffleBoard
    SequentialCommandGroup backToBalAndBalTest = new SequentialCommandGroup(robotDrive.AutoCommandFactory(Trajectories.backToBalTest), new SwerveControllerCmd(robotDrive, true));

    SwerveControllerCmd blank = new SwerveControllerCmd(
      robotDrive,
      () -> 0.0,
      () -> 0.0,
      () -> 0.0,
      () -> true);

    //autoChooser.setDefaultOption("Drive Back Blue", driveBackAutoBlue);
    //autoChooser.addOption("Drive back Red", driveBackAutoRed);
    //autoChooser.addOption("Prepare To Pounce", prepareToPounceTest);
    autoChooser.addOption("Void", blank);
    //autoChooser.addOption("Blue Top To Cargo", blueTopToChargeCmd);
    autoChooser.addOption("[TEST] Auto Balance", new SwerveControllerCmd(robotDrive, true));
    autoChooser.addOption("Test Auto Balance", new SwerveControllerCmd(robotDrive, true));
    //autoChooser.addOption("Drive Back Retract Blue", driveBackRetractBlueLeft);
    //autoChooser.addOption("Charge Up test", blueTopToChargeUpCmd);
    autoChooser.addOption("21 PT Blue Left", fullTwentyOnePtAutonomousBlueLeft);
    autoChooser.addOption("21 PT Blue Right", fullTwentyOnePtAutonomousBlueRight);
    autoChooser.addOption("21 PT Red Right", fullTwentyOnePtAutonomousRedRight);
    autoChooser.addOption("21 PT Red Left", fullTwentyOnePtAutonomousRedLeft);
    autoChooser.addOption("Single Score Final", singleScoreAuto);
    autoChooser.addOption("Double Scoring Auto", doubleScoringAuto);    

    autoChooser.addOption("[TEST] S Curve Auto", S_Curve);
    autoChooser.addOption("Single Score Drive Back Blue Left", singleScoreBlueThenBackLeft);
    autoChooser.addOption("Single Score Drive Back Red Left", singleScoreRedThenBackLeft);
    autoChooser.addOption("Single Score Drive Back Blue Right", singleScoreBlueThenBackRight);
    autoChooser.addOption("Single Score Drive Back Red Right", singleScoreRedThenBackRight);

    autoChooser.addOption("[TEST] Capture Cargo Test", captureCargo2);
    autoChooser.addOption("[TEST] Back and bal", backToBalAndBalTest);

    SmartDashboard.putData(autoChooser);
/* 
    lightingChooser.addOption("Rainbow", new rainbow(lightingSubsystem));
    lightingChooser.addOption("Rainbow From Mid", new rainbowFromMid(lightingSubsystem));
    lightingChooser.addOption("BiPride", new BiPrideCmd(lightingSubsystem));
    lightingChooser.setDefaultOption("CargoSignal", new CargoSignalCmd(lightingSubsystem, pneumaticsSubsystem));
    lightingChooser.addOption("Trans Pride", new TransPrideCmd(lightingSubsystem));

    SmartDashboard.putData(lightingChooser);
*/
  
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
    new JoystickButton(operatorController, Buttons.A).whileTrue(new ClawCmd(robotManipulator, 
      () -> ManipulatorConstants.kClawMotorSpeed, () -> 1));
    
    new JoystickButton(operatorController, Buttons.Y).whileTrue(new ClawCmd(robotManipulator, 
      () -> ManipulatorConstants.kClawMotorSpeed, () -> -1));

    new JoystickButton(driverController, Buttons.LB).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    //new JoystickButton(driverController, Buttons.RB).whileTrue(new RumbleCmd(operatorController, 2, 1.00));

    
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));
    


    new JoystickButton(driverController, Buttons.RB).toggleOnTrue(new SullyCmd(pneumaticsSubsystem));

    new POVButton(operatorController, Buttons.LEFT_ARR).toggleOnTrue(new SallyCmd(pneumaticsSubsystem));

    new POVButton(operatorController, Buttons.UP_ARR).whileTrue(new IntakeCmd(robotManipulator, 
    () -> (0.25), -1));



    //New Telescope with parallel arm processing
    //Extend Telescope
    new JoystickButton(operatorController, Buttons.LB).whileTrue(new TelescopeWithLiftBoundless(robotLift, ()-> LiftConstants.kTelescopeSpeed, () -> 1,
      () -> operatorController.getLeftTriggerAxis(),
      () -> operatorController.getRightTriggerAxis(), false));
    
    //Retract Telescopes
    new JoystickButton(operatorController, Buttons.RB).whileTrue(new TelescopeWithLiftBoundless(robotLift, ()-> LiftConstants.kTelescopeSpeed, () -> -1, 
      () -> operatorController.getLeftTriggerAxis(),
      () -> operatorController.getRightTriggerAxis(), false));
    

    //Move arm up to optimal intake setpoint
    new POVButton(operatorController, Buttons.DOWN_ARR).whileTrue(new LiftAutoCmd(robotLift, LiftConstants.kIntakeLiftPosition));

    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new LiftAutoCmd(robotLift, LiftConstants.kConeShelfSetPoint));

    //Cone shelf setpoint
    //new JoystickButton(operatorController, Buttons.R3).toggleOnTrue(new LiftAutoCmd(robotLift, LiftConstants.kConeShelfSetPoint));

    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Rotate robot to pick up cube
    new JoystickButton(driverController, Buttons.Y).whileTrue(new SwerveControllerCmd(robotDrive, () -> (0.0), () -> (0.0), 
      () -> (-MathMethods.speedMax2(0.04*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
      () -> false));

    //Move robot to pick up cone
    new JoystickButton(driverController, Buttons.B).whileTrue(new ParallelCommandGroup(new SwerveControllerCmd(robotDrive, () -> (0.15), () -> (0.0), 
      () -> (-MathMethods.speedMax2(0.045*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
      () -> false), new ClawCmd(robotManipulator, ()->0.7, ()->1)));
    


    //Reset lift bounds in case of error
    //new JoystickButton(driverController, Buttons.B).toggleOnTrue(new ResetLiftBoundsCmd(robotLift));

    //Slow drive with d-pad
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true));


    //AutoBalance in teleop using button
      new JoystickButton(driverController, Buttons.A).whileTrue(new SwerveControllerCmd(robotDrive, 
      () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees, AutoConstants.kAutoBalanceMinSpeed)),
      () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees, AutoConstants.kAutoBalanceMinSpeed)),
      () -> (0.0), () -> false));
      
    //Experimental AutoBalance PLEASE TEST
    //new JoystickButton(driverController, Buttons.A).whileTrue(new SwerveControllerCmd(robotDrive, true));

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return the autonomous command given by the drop-down selector in ShuffleBoard
    return autoChooser.getSelected();

  }
/* 
  public Command getLightsCommand() {
    return lightingChooser.getSelected();
  }
*/
}
