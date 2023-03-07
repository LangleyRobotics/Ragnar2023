package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;


//UNFINISHED
/*Planned changes:
    - Log botpose over a short time interval
    - Design "median" algorithm to find stable botpose 
    - Will enable resilience against tracking gaps and pose ambiguity
*/

public class AprilReorientation extends CommandBase{
    private final LimelightSubsystem limelightSubsystem;


    public AprilReorientation(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] aprilBotPose = limelightSubsystem.get_LL_botpose();
        //Command Test One: Reorients bot to field pose of zero, then zeros the heading. 
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
