package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MathMethods;
import frc.robot.subsystems.LiftSubsystem;



public class LiftAutoCmd extends CommandBase{
    
    private final LiftSubsystem liftSubsystem;
    private final double setPoint;
    private Boolean isFinished;


    public LiftAutoCmd(LiftSubsystem liftSubsystem, double setPoint) {
        this.liftSubsystem = liftSubsystem;
        this.setPoint = setPoint;
        //this.isFinished = isFinished;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;

    }

    @Override
    public void execute() {
        double liftPos = liftSubsystem.getLiftAbsEncoder();
        double distToSetPoint = setPoint - liftPos;
        if (distToSetPoint!=0 && Math.abs(distToSetPoint) < 0.05) {
            liftSubsystem.setLiftMotor((-MathMethods.signDouble(distToSetPoint)*0.35));
        }
        if (Math.abs(distToSetPoint)>=0.05 && Math.abs(distToSetPoint)<=0.10) {
            liftSubsystem.setLiftMotor((-8*distToSetPoint));
        }
        if (Math.abs(distToSetPoint) > 0.10) {
            liftSubsystem.setLiftMotor(-MathMethods.signDouble(distToSetPoint)*0.80);
        }

        if (Math.abs(distToSetPoint) < 0.004) {
            liftSubsystem.stopLiftMotor();
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (isFinished) {
            return true;
        } else {
            return false;
        }
    }


}
