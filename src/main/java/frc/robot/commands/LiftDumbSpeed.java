package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.MathMethods;
import frc.robot.Constants.LiftConstants;

import frc.robot.subsystems.LiftSubsystem;


public class LiftDumbSpeed extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final double velocity;

    public LiftDumbSpeed(LiftSubsystem liftSubsystem, double velocity) {
        this.liftSubsystem = liftSubsystem;
        this.velocity = velocity;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        liftSubsystem.setLiftMotor(velocity);
        
    } 

    @Override
    public void end(boolean interrupted) {
        //liftSubsystem.stopLiftMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
