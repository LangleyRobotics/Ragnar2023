package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;


//Custum holonic drive controller
//PID controller for each direction
//Profiled PID controller for the angular direction


public class CustomHolonomicDrive {
    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseTolerance = new Pose2d(0.05, 0.05, new Rotation2d(Math.toRadians(0.2)));
    private double kControlFactorTranslation = 0.2;
    private double kControlFactorRotation = 0.2;
    
    private final PIDController m_XYController;
    private final PIDController m_ThetaController;


    public CustomHolonomicDrive(PIDController xyController, PIDController thetaController) {
        m_XYController = xyController;
        m_ThetaController = thetaController;
        m_ThetaController.enableContinuousInput(-180, 180);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        m_poseError = targetPose.relativeTo(currentPose);

        double xFeedback = m_XYController.calculate(currentPose.getX(), targetPose.getX());
        double yFeedback = m_XYController.calculate(currentPose.getY(), targetPose.getY());
        double thetaFeedback = m_ThetaController.calculate(currentPose.getRotation().getDegrees(),
            targetPose.getRotation().getDegrees());

        double x = xFeedback;
        double y = yFeedback;

        double mag = Math.sqrt((x * x) + (y * y));

        //double xF = x / mag;
        //double yF = y / mag;

        return ChassisSpeeds.fromFieldRelativeSpeeds(x * Constants.kMaxSpeedMetersPerSecond,
                                                        y * Constants.kMaxSpeedMetersPerSecond,
                                                        thetaFeedback * 2.0,
                                                        currentPose.getRotation());
    }
    
}