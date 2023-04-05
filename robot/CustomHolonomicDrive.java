package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


//Custom holonomic drive controller
//PID controller for translation
//PID controller for rotation

public class CustomHolonomicDrive {
    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseDeadband = new Pose2d(0.1, 0.1, new Rotation2d(Math.toRadians(0.8)));
    
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

        return ChassisSpeeds.fromFieldRelativeSpeeds(x * Constants.kMaxSpeedMetersPerSecond,
                                                        y * Constants.kMaxSpeedMetersPerSecond,
                                                        thetaFeedback * 1.5,
                                                        currentPose.getRotation());
    }

    public boolean targetPoseAchieved() {
        final var translationError = m_poseError.getTranslation();
        final var translationDeadband = m_poseDeadband.getTranslation();
        final var rotationError = m_poseError.getRotation();
        final var rotationDeadband = m_poseDeadband.getRotation();
        return Math.abs(translationError.getX()) < translationDeadband.getX()
            && Math.abs(translationError.getY()) < translationDeadband.getY()
            && Math.abs(rotationError.getDegrees()) < rotationDeadband.getDegrees();
    }
    
}