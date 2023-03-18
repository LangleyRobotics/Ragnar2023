package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


//UNFINISHED
/*Planned changes:
    - Log botpose over a short time interval
    - Design "median" algorithm to find stable botpose 
    - Will enable resilience against tracking gaps and pose ambiguity
*/

public class AprilReorientation extends CommandBase{
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;


    public AprilReorientation(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(limelightSubsystem, driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] aprilBotPoseArr = limelightSubsystem.get_LL_botpose();

        double aprilPoseX = aprilBotPoseArr[0];
        double aprilPoseY = aprilBotPoseArr[1];
        double aprilYaw = aprilBotPoseArr[5];

        driveSubsystem.setPose(new Pose2d(aprilPoseX, aprilPoseY,new Rotation2d(aprilYaw)));
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
