package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.epilepsySubsystem;


public class prideMonth extends CommandBase{
    private final epilepsySubsystem epilepsySubsystem;


    public prideMonth(epilepsySubsystem epilepsySubsystem) {
        this.epilepsySubsystem = epilepsySubsystem;
        addRequirements(epilepsySubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        epilepsySubsystem.Rainbow();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }    


   /*     epilepsySubsystem.transPride();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        epilepsySubsystem.turnOff();
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
