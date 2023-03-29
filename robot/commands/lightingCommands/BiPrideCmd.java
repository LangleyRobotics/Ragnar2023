package frc.robot.commands.lightingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lightingSubsystem;


public class BiPrideCmd extends CommandBase{
    private final lightingSubsystem lightingSubsystem;


    public BiPrideCmd(lightingSubsystem lightingSubsystem) {
        this.lightingSubsystem = lightingSubsystem;
        addRequirements(lightingSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        lightingSubsystem.biPride();
    }
     
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
