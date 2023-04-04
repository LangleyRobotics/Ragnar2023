package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.MathMethods;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.TelescopeWithLiftCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class CaptureCargo {


    public static ParallelRaceGroup getSequentialCommandGroup(DriveSubsystem robotDrive, LimelightSubsystem limelightSubsystem, 
        ManipulatorSubsystem robotManipulator, LiftSubsystem robotLift) {
        return new ParallelRaceGroup(new SwerveControllerCmd(robotDrive, () -> (0.30), () -> (0.0), 
            () -> (-MathMethods.speedMax2(0.04*limelightSubsystem.getTargetOffsetX(), 0.2, 0.05)),
            () -> false).withTimeout(1.5), 
            new IntakeCmd(robotManipulator, 
            () -> ManipulatorConstants.kIntakeMotorSpeed, -1),
            new TelescopeWithLiftCmd(robotLift, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> 0.0, () -> 0.0, false));
  
    } 

}
