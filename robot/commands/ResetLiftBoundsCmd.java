package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class ResetLiftBoundsCmd extends CommandBase{
    private final LiftSubsystem liftSubsystem;

    public ResetLiftBoundsCmd(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    } 

    @Override
    public void initialize() {
        liftSubsystem.resetLiftAbsEncoder();
    }

    @Override
    public void execute() {
        

    }

       
    @Override
    public void end(boolean interrupted) {
        

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
