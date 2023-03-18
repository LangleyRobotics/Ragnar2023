package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.LiftSubsystem;

public class TelescopeWithLiftBoundless extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final Supplier<Double> telescopeSpeed;
    private final Supplier<Integer> telescopeDirection;

    private final Supplier<Double> liftPositiveSpdFunction;
    private final Supplier<Double> liftNegativeSpdFunction;
    private final Boolean returnVal;


    public TelescopeWithLiftBoundless(LiftSubsystem liftSubsystem, Supplier<Double> telescopeSpeed, Supplier<Integer> telescopeDirection, 
        Supplier<Double> liftPositiveSpdFunction, Supplier<Double> liftNegativeSpdFunction, Boolean returnVal) {
        this.liftSubsystem = liftSubsystem;
        this.telescopeSpeed = telescopeSpeed;
        this.telescopeDirection = telescopeDirection;
        this.liftPositiveSpdFunction = liftPositiveSpdFunction;
        this.liftNegativeSpdFunction = liftNegativeSpdFunction;
        this.returnVal = returnVal;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double teleSpd = telescopeSpeed.get();
        int teleDir = telescopeDirection.get();
        double teleVel = teleSpd*teleDir;
        liftSubsystem.setTelescopeMotor(teleVel);

        double liftPositiveSpeed = liftPositiveSpdFunction.get();
        double liftNegativeSpeed = liftNegativeSpdFunction.get();

        double liftVel = liftPositiveSpeed - liftNegativeSpeed;
        liftSubsystem.setLiftMotorNoBounds(0.5*liftVel);
        


    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopTelescopeMotor();
        liftSubsystem.stopLiftMotor();
    }

    @Override
    public boolean isFinished() {
        if (!returnVal) {
            return false;
        } else {
            return liftSubsystem.getLimitSwitchInput();
        }
    }


}
