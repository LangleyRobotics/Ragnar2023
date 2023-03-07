package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSubsystem;



public class LiftIntakeCmd extends CommandBase{
    
    private final LiftSubsystem liftSubsystem;


    public LiftIntakeCmd(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {

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

        if (distToIntake > 300) {
            liftSubsystem.setLiftMotor(-0.3);
        }

        if (Math.abs(distToIntake) < 2) {
            liftSubsystem.stopLiftMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
