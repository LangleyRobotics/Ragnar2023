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
import frc.robot.subsystems.lightingSubsystem;

public class RetractManipSeqCmdGrp {


    public static SequentialCommandGroup getSequentialCommandGroup(LiftSubsystem liftSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return new SequentialCommandGroup(
            new TelescopeWithLiftCmd(liftSubsystem, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> -0.3, () -> 0.0, false).withTimeout(1.5),
            new TelescopeWithLiftCmd(liftSubsystem, () -> LiftConstants.kTelescopeSpeed, () -> -1, () -> 0.90, () -> 0.0, false).withTimeout(2.5), 
            new SullyCmd(pneumaticsSubsystem));
    } 

}
