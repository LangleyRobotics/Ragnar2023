package frc.robot.commands.AutoCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MathMethods;

import frc.robot.subsystems.LiftSubsystem;




public class LiftAutoCmdPID extends CommandBase{
    private final LiftSubsystem liftSubsystem;
    private final double setPoint;
    private final PIDController pController;


    public LiftAutoCmdPID(LiftSubsystem liftSubsystem, double setPoint) {
        this.liftSubsystem = liftSubsystem;
        this.setPoint = setPoint;
        this.pController = new PIDController(MathMethods.signDouble(liftSubsystem.getLiftAbsEncoder()-setPoint)*3.5, 0.1, 0);

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        pController.setTolerance(0.005);
    }

    @Override
    public void execute() {
        liftSubsystem.setLiftMotor(pController.calculate(liftSubsystem.getLiftAbsEncoder() , setPoint));
        //pController.setP((liftSubsystem.getLiftAbsEncoder()-setPoint)*20.0);
    } 

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLiftMotor();
    }

    @Override
    public boolean isFinished() {
        return pController.atSetpoint();
    }


}
