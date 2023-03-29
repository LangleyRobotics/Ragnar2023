package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.ClawCmd;
import frc.robot.commands.SullyCmd;
import frc.robot.commands.TelescopeCmd;
import frc.robot.commands.TelescopeWithLiftCmd;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class outtakeAutoSeqCmdGrp {


    public static SequentialCommandGroup getSequentialCommandGroup(LiftSubsystem liftSubsystem, ManipulatorSubsystem manipulatorSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return new SequentialCommandGroup(new SullyCmd(pneumaticsSubsystem), new WaitCommand(1.0), 
            new TelescopeWithLiftCmd(liftSubsystem, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.90, () -> 0.0, false).withTimeout(2),
            new TelescopeWithLiftCmd(liftSubsystem, () -> LiftConstants.kTelescopeSpeed, () -> 1, () -> -0.3, () -> 0.0, false).withTimeout(0.2),
            new TelescopeCmd(liftSubsystem, () -> 0.0, () -> 1).withTimeout(0.1),
            new ClawCmd(manipulatorSubsystem, () -> ManipulatorConstants.kClawMotorSpeed, ()-> -1).withTimeout(0.5),
            new ClawCmd(manipulatorSubsystem, () -> ManipulatorConstants.kClawMotorSpeed, ()-> 1).withTimeout(0.5));
    } 

}
