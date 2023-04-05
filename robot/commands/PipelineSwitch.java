package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LimelightSubsystem;

public class PipelineSwitch extends CommandBase{
    private final LimelightSubsystem limelightSubsystem;
    private final int mode; 

    public PipelineSwitch(LimelightSubsystem limelightSubsystem, int mode) {
        this.limelightSubsystem = limelightSubsystem;
        this.mode = mode;

        addRequirements(limelightSubsystem);
    }

    public PipelineSwitch(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.mode = 10;

        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        int temp2 = mode;
        if (mode > 9) {
            int temp = (int)limelightSubsystem.getPipeline();
            if (temp == 1) {
                temp2 = 0;
            } else {
                temp2 = 1;
            }
        }
        limelightSubsystem.setPipeline(temp2);
        limelightSubsystem.setModeVision();
    }

    @Override
    public void execute() {
        

    
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }



}
