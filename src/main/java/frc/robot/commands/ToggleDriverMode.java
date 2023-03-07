package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class ToggleDriverMode extends CommandBase{
    private final LimelightSubsystem limelight;
    public ToggleDriverMode(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }
    
    @Override
    public void initialize() {
        limelight.setModeDriver();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
    
}
