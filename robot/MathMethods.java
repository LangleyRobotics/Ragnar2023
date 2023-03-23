package frc.robot;

import frc.robot.Constants.AutoConstants;

public class MathMethods {
    public static final double Tau = 2*Math.PI;
    public static double moduloAngle(double angle) {
        while (angle > Math.PI) {
            angle -= Tau;
        }
        while (angle < -Math.PI) {
            angle += Tau;
        }

        return angle;
    }

    //Define acceleration paramaters for auto-level
    //Use  AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond as accelCap to implement in balance funtion
    public static double speedMax(double initVel, double maxSpeed, double deadband) 
    {
        if (Math.abs(initVel) > maxSpeed) {
            return maxSpeed*((Math.abs(initVel))/initVel);
        } else if (Math.abs(initVel) < AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond*Math.sin(Math.toRadians(deadband))) {
            return 0;
        } else {
            return initVel;
        }
    }

    public static double speedMax2(double initVel, double maxSpeed, double deadband) 
    {
        if (Math.abs(initVel) > maxSpeed) {
            return maxSpeed*((Math.abs(initVel))/initVel);
        } else if (Math.abs(initVel) < deadband) {
            return 0;
        } else {
            return initVel;
        }
    }

    public static int signDouble(double num) {
        return (int) (Math.abs(num)/num);
    }
    
}
