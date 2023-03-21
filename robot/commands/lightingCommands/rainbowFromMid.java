package frc.robot.commands.lightingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lightingSubsystem;


public class rainbowFromMid extends CommandBase{
    private final lightingSubsystem lightingSubsystem;


    public rainbowFromMid(lightingSubsystem lightingSubsystem) {
        this.lightingSubsystem = lightingSubsystem;
        addRequirements(lightingSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        lightingSubsystem.rainbowFromCenter();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }    


   /*     lightingSubsystem.transPride();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        lightingSubsystem.turnOff();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        */
    }
     
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
