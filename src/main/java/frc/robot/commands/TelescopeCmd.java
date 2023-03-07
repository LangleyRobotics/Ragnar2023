package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;

import frc.robot.subsystems.LiftSubsystem;

public class TelescopeCmd extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final Supplier<Double> telescopeSpeed;
    private final Supplier<Integer> telescopeDirection;

    public TelescopeCmd(LiftSubsystem liftSubsystem, Supplier<Double> telescopeSpeed, Supplier<Integer> telescopeDirection) {
        this.liftSubsystem = liftSubsystem;
        this.telescopeSpeed = telescopeSpeed;
        this.telescopeDirection = telescopeDirection;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double speed = telescopeSpeed.get();
        int direction = telescopeDirection.get();
        double velocity = speed*direction;
        liftSubsystem.setTelescopeMotor(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopTelescopeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
