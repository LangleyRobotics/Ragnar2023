// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
//import frc.robot.commands.LiftAutoCmd;
import frc.robot.commands.LiftIntakeCmd;
import frc.robot.commands.LiftTriggerCmd;
//import frc.robot.commands.ResetLiftBoundsCmd;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.TelescopeCmd;
import frc.robot.commands.TelescopeWithLiftBoundless;
import frc.robot.commands.TelescopeWithLiftCmd;
import frc.robot.commands.outtakeAutoSeqCmdGrp;
import frc.robot.commands.prideMonth;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.SallyCmd;
import frc.robot.commands.SullyCmd;

//Subsystem Imports
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.epilepsySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private final epilepsySubsystem epilepsySubsystem = new epilepsySubsystem();



  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);

  //COMMENT OUT THE CONTROLLER BELOW - ONLY USE WHEN NEED PS4 CONTROLLER
  //PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
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

    epilepsySubsystem.setDefaultCommand(new prideMonth(epilepsySubsystem));


    //Load in paths from Trajectories as drive commands using the AutoCommandFactory
    SequentialCommandGroup S_Curve = robotDrive.AutoCommandFactory(Trajectories.defaultTrajectory);

    SequentialCommandGroup blueTopToChargeCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopToCharge);
    SequentialCommandGroup blueTopToChargeCmd2 = robotDrive.AutoCommandFactory(Trajectories.blueTopToCharge);
    SequentialCommandGroup blueTopToChargeUpCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUp);
    SequentialCommandGroup blueTopToChargeUpCmd2 = robotDrive.AutoCommandFactory(Trajectories.blueTopToChargeUp);

    SequentialCommandGroup blueTopTo2CargoCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopTo2Cargo);
    SequentialCommandGroup blueTopCargoReturnCmd = robotDrive.AutoCommandFactory(Trajectories.blueTopReturnCargo);

    SequentialCommandGroup redTopToChargeCmd = robotDrive.AutoCommandFactory(Trajectories.redTopToCharge);
    SequentialCommandGroup redTopToChargeCmd2 = robotDrive.AutoCommandFactory(Trajectories.redTopToCharge);
    SequentialCommandGroup redTopToChargeUpCmd = robotDrive.AutoCommandFactory(Trajectories.redTopToChargeUp);
    SequentialCommandGroup redTopToChargeUpCmd2 = robotDrive.AutoCommandFactory(Trajectories.redTopToChargeUp);

    

    //Create telescope and lift commands
    ParallelRaceGroup teleAutoRetractWithArmResist = new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(0.2);
    ParallelRaceGroup armDownTele = new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5);

    //Create piston toggle command
    SullyCmd sullyTheMandalorian = new SullyCmd(pneumaticsSubsystem);

    //Create command sequence that stows telescope, lift, and drop-down intake
    SequentialCommandGroup retractManipulators = new SequentialCommandGroup(teleAutoRetractWithArmResist, armDownTele, sullyTheMandalorian);
    SequentialCommandGroup retractManipulators2 = new SequentialCommandGroup(
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
      new SullyCmd(pneumaticsSubsystem));
    SequentialCommandGroup retractManipulators3 = new SequentialCommandGroup(
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
      new SullyCmd(pneumaticsSubsystem));

    //Create parallel commands to perform manipulator stow while driving
    ParallelCommandGroup driveBackRetractBlue = new ParallelCommandGroup(retractManipulators3, blueTopToChargeCmd);
    ParallelCommandGroup toCargo2AndRetract = new ParallelCommandGroup(retractManipulators2, blueTopTo2CargoCmd);

    ParallelCommandGroup driveBackRetractRed = new ParallelCommandGroup(retractManipulators, redTopToChargeCmd);

    //Create auto balance command
    SwerveControllerCmd autoBalance = new SwerveControllerCmd(robotDrive, 
      () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
      () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
      () -> (0.0), () -> false);

    SwerveControllerCmd autoBalance2 = new SwerveControllerCmd(robotDrive, 
      () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
      () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
      () -> (0.0), () -> false);



    //21 pt autonomous - Score preloaded cargo, drive around the community boundary to the charge plate, balance on charge plate
    SequentialCommandGroup fullTwentyOnePtAutonomousBlue = new SequentialCommandGroup(
      new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(2),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
      new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
      new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
      new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5), 
      driveBackRetractBlue).andThen(blueTopToChargeUpCmd, autoBalance);

      SequentialCommandGroup fullTwentyOnePtAutonomousRed = new SequentialCommandGroup(
        new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(2),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
        new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5), 
        driveBackRetractRed).andThen(redTopToChargeUpCmd, autoBalance2);


      
      SequentialCommandGroup chargeUpAndBalanceAutoRed = new SequentialCommandGroup(redTopToChargeCmd2, redTopToChargeUpCmd2, new SwerveControllerCmd(robotDrive, 
        () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (0.0), () -> false));

      SequentialCommandGroup chargeUpAndBalanceAutoBlue = new SequentialCommandGroup(blueTopToChargeCmd2, blueTopToChargeUpCmd2, new SwerveControllerCmd(robotDrive, 
        () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (0.0), () -> false));

    //Unfinished 15 pt autonomous - Score preloaded cargo, drive out of commmunity boundary to pick up cargo, drive back to cube node, score new cargo
    SequentialCommandGroup doubleScoringAuto = new SequentialCommandGroup(
        new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(2),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
        new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5), 
        toCargo2AndRetract,
        new SullyCmd(pneumaticsSubsystem).withTimeout(0.75),
        new LiftIntakeCmd(robotLift), 
        new IntakeCmd(robotManipulator, ()-> ManipulatorConstants.kIntakeMotorSpeed, 1).withTimeout(0.5),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(1.5),//Arm Up
        new SullyCmd(pneumaticsSubsystem),
        new ParallelCommandGroup(
          /*new SequentialCommandGroup(new WaitCommand(0.6), new AprilReorientation(limelightSubsystem, robotDrive)),*/ 
          new ParallelRaceGroup(blueTopCargoReturnCmd, new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false))),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(2.0),
        new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5));

    
    
    SequentialCommandGroup singleScoreBothColors = new SequentialCommandGroup(
      new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(2),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
      new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
      new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5),
      new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
      new SullyCmd(pneumaticsSubsystem));


      SequentialCommandGroup singleScoreBlueThenBack = new SequentialCommandGroup(
        new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.90, () -> 0.0, false).withTimeout(2),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
        new TelescopeCmd(robotLift, () -> 0.0, () -> 1).withTimeout(0.1),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5),
        new ClawCmd(robotManipulator, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
        new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
        new SullyCmd(pneumaticsSubsystem),robotDrive.AutoCommandFactory(Trajectories.backAutoBlue));


    SequentialCommandGroup singleScoreEffTest = new SequentialCommandGroup(outtakeAutoSeqCmdGrp.getSequentialCommandGroup(robotLift, robotManipulator, pneumaticsSubsystem),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
      new TelescopeWithLiftCmd(robotLift, () -> 1.00, () -> 1, () -> 0.90, () -> 0.0, false).withTimeout(2.5),
      new SullyCmd(pneumaticsSubsystem));


    SequentialCommandGroup driveBackAutoRed = robotDrive.AutoCommandFactory(Trajectories.backAutoRed);
    SequentialCommandGroup driveBackAutoBlue = robotDrive.AutoCommandFactory(Trajectories.backAutoBlue);
    //Add autonomous commands to drop-down selector on ShuffleBoard
    
    SwerveControllerCmd blank = new SwerveControllerCmd(
      robotDrive,
      () -> 0.0,
      () -> 0.0,
      () -> 0.0,
      () -> true);
    autoChooser.setDefaultOption("Drive Back Blue", driveBackAutoBlue);
    autoChooser.addOption("Drive back Red", driveBackAutoRed);
    autoChooser.addOption("Void", blank);
    autoChooser.addOption("Blue Top To Cargo", blueTopToChargeCmd);
    autoChooser.addOption("Auto Balance", autoBalance);
    autoChooser.addOption("Drive Back Retract Blue", driveBackRetractBlue);
    autoChooser.addOption("Charge Up test", blueTopToChargeUpCmd);
    autoChooser.addOption("21 PT Blue", fullTwentyOnePtAutonomousBlue);
    autoChooser.addOption("21 PT Red", fullTwentyOnePtAutonomousRed);
    autoChooser.addOption("Single Score Both Colors", singleScoreBothColors);
    autoChooser.addOption("Single Eff TEST", singleScoreEffTest);
    autoChooser.addOption("Double Scoring Auto", doubleScoringAuto);    
    autoChooser.addOption("Charge up and bal Blue", chargeUpAndBalanceAutoBlue);
    autoChooser.addOption("Charge up and bal Red", chargeUpAndBalanceAutoRed);
    autoChooser.addOption("S Curve Auto", S_Curve);
    autoChooser.addOption("Single Score Drive Back Blue", singleScoreBlueThenBack);



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
    
    new POVButton(operatorController, Buttons.LEFT_ARR).whileTrue(new DropDownExclusiveCmd(robotManipulator, 
      () -> 0.3, -1));

    new JoystickButton(driverController, Buttons.LB).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    //new JoystickButton(driverController, Buttons.RB).whileTrue(new RumbleCmd(operatorController, 2, 1.00));

    
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));
    


    new JoystickButton(driverController, Buttons.RB).toggleOnTrue(new SullyCmd(pneumaticsSubsystem));

    new POVButton(operatorController, Buttons.LEFT_ARR).toggleOnTrue(new SallyCmd(pneumaticsSubsystem));



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
    new POVButton(operatorController, Buttons.DOWN_ARR).toggleOnTrue(new LiftIntakeCmd(robotLift));

    //Cone shelf setpoint
    //new JoystickButton(operatorController, Buttons.R3).toggleOnTrue(new LiftAutoCmd(robotLift, LiftConstants.kConeShelfSetPoint));

    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Reset lift bounds in case of error
    //new JoystickButton(driverController, Buttons.B).toggleOnTrue(new ResetLiftBoundsCmd(robotLift));

    //Slow drive with d-pad
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true));
    new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true));


    //AutoBalance in teleop using button
      new JoystickButton(driverController, Buttons.A).whileTrue(new SwerveControllerCmd(robotDrive, 
        () -> (MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getPitch())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (-MathMethods.speedMax(AutoConstants.kAutoBalanceSpeedFactor*Math.sin(Math.toRadians(robotDrive.getRoll())), AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond, AutoConstants.kAutoBalanceDeadbandDegrees)),
        () -> (0.0), () -> false));

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
}
