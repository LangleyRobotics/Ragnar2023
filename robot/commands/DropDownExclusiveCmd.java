package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import frc.robot.subsystems.ManipulatorSubsystem;

public class DropDownExclusiveCmd extends CommandBase{
    private final ManipulatorSubsystem manipulatorSubsystem;
    private final Supplier<Double> speedSupplier;
    //direction int is negative 1 or 1.
    private final int direction;

    public DropDownExclusiveCmd(ManipulatorSubsystem manipulatorSubsystem, Supplier<Double> speedSupplier, int direction) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.speedSupplier = speedSupplier;
        this.direction = direction;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = speedSupplier.get();
        int dir = direction;
        double velocity = speed*dir;

        manipulatorSubsystem.setIntakeMotor(velocity);
    
    
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
