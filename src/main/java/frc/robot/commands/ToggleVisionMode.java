package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class ToggleVisionMode extends CommandBase{
    private final LimelightSubsystem limelight;
    public ToggleVisionMode(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }
    
    @Override
    public void initialize() {
        limelight.setModeVision();
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
