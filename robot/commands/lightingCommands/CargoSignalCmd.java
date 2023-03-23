package frc.robot.commands.lightingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.lightingSubsystem;


public class CargoSignalCmd extends CommandBase{
    private final lightingSubsystem lightingSubsystem;
    private final PneumaticsSubsystem pneumaticsSubsystem;


    public CargoSignalCmd(lightingSubsystem lightingSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        this.lightingSubsystem = lightingSubsystem;
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(lightingSubsystem);
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (pneumaticsSubsystem.isSallyAwake()) {
            //set purple
            lightingSubsystem.setAllColorUniform(255, 0, 255);
            //95, 52, 206
        }
        if (!pneumaticsSubsystem.isSallyAwake()) {
            //set yellow
            lightingSubsystem.setAllColorUniform(250, 150, 0);
        }

    }
     
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
