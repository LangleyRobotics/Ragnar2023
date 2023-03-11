package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MathMethods;
import frc.robot.Constants.LiftConstants;

import frc.robot.subsystems.LiftSubsystem;


public class LiftAutoCmd extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final double setPoint;
    private Boolean isFinished;


    public LiftAutoCmd(LiftSubsystem liftSubsystem, double setPoint) {
        this.liftSubsystem = liftSubsystem;
        this.setPoint = setPoint;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {

        double distFromSetpoint = liftSubsystem.getLiftAbsEncoder() - setPoint;
        double velocity;
        if (Math.abs(distFromSetpoint)>100)
            velocity = MathMethods.signDouble(distFromSetpoint)*0.5;
        else {
            velocity = 0.5*LiftConstants.kliftSetPointSpeedConversionFactorClose*distFromSetpoint;
        }

        if (Math.abs(distFromSetpoint) > 10) {
            liftSubsystem.setLiftMotor(velocity);
            isFinished = true;
        } else {
            end(true);
        }
        
    } 

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLiftMotor();
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
