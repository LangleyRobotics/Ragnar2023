package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSubsystem;



public class LiftIntakeCmd extends CommandBase{
    
    private final LiftSubsystem liftSubsystem;
    private Boolean isFinished;


    public LiftIntakeCmd(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        //this.isFinished = isFinished;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;

    }

    @Override
    public void execute() {
        TrapezoidProfile liftIntakeProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(5, 0),
                                                new TrapezoidProfile.State(0, 0));

        double liftPos = liftSubsystem.getLiftAbsEncoder();
        double distToIntake = LiftConstants.kIntakeLiftPosition - liftPos;
        if (distToIntake != 0 && Math.abs(distToIntake)<100) {
            liftSubsystem.setLiftMotor(-0.003*(distToIntake));
        }

        if (distToIntake > 100) {
            liftSubsystem.setLiftMotor(-0.3);
        }

        if (Math.abs(distToIntake) < 10) {
            liftSubsystem.stopLiftMotor();
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (isFinished) {
            return true;
        } else {
            return false;
        }
    }


}
