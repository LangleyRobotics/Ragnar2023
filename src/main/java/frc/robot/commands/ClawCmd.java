package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

import frc.robot.subsystems.ManipulatorSubsystem;

public class ClawCmd extends CommandBase{
    private final ManipulatorSubsystem manipulatorSubsystem;
    private final Supplier<Double> speedSupplier;
    //direction int is negative 1 or 1.
    private final Supplier<Integer> directionSupplier;

    public ClawCmd(ManipulatorSubsystem manipulatorSubsystem, Supplier<Double> speedSupplier, Supplier<Integer> directionSupplier) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.speedSupplier = speedSupplier;
        this.directionSupplier = directionSupplier;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = speedSupplier.get();
        int direction = directionSupplier.get();
        double velocity = speed*direction;

        manipulatorSubsystem.setClawMotor(velocity);

    
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorSubsystem.stopClawMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
