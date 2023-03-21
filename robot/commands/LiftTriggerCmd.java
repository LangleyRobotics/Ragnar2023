package frc.robot.commands;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LiftSubsystem;



public class LiftTriggerCmd extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final Supplier<Double> liftPositiveSpdFunction;
    private final Supplier<Double> liftNegativeSpdFunction;


    public LiftTriggerCmd(LiftSubsystem liftSubsystem, Supplier<Double> liftPositiveSpdFunction, Supplier<Double> liftNegativeSpdFunction) {
        this.liftSubsystem = liftSubsystem;
        this.liftPositiveSpdFunction = liftPositiveSpdFunction;
        this.liftNegativeSpdFunction = liftNegativeSpdFunction;

     

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double positiveSpeed = liftPositiveSpdFunction.get();
        double negativeSpeed = liftNegativeSpdFunction.get();

        double velocity = positiveSpeed - negativeSpeed;
        liftSubsystem.setLiftMotorNoBounds(0.5*velocity);
    } 

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLiftMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
