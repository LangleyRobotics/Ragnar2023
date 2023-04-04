package frc.robot.subsystems;

import frc.robot.Constants.LiftConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MathMethods;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.revrobotics.CANSparkMax;


public class LiftSubsystem extends SubsystemBase{

    private final CANSparkMax liftMotorLeft = new CANSparkMax(LiftConstants.kLiftMotorLeft, MotorType.kBrushless);
    private final CANSparkMax liftMotorRight = new CANSparkMax(LiftConstants.kLiftMotorRight, MotorType.kBrushless);
    private final CANSparkMax telescopeMotor = new CANSparkMax(LiftConstants.kTelescopeMotor, MotorType.kBrushless);
    private final DutyCycleEncoder liftAbsEncoder = new DutyCycleEncoder(1);
    private final RelativeEncoder liftRelEncoder = liftMotorLeft.getAlternateEncoder(8192);
    private final RelativeEncoder telescopeEnc = telescopeMotor.getEncoder();
    private final ArmFeedforward liftForwardController = new ArmFeedforward(LiftConstants.kS_Lift, LiftConstants.kG_Lift, LiftConstants.kV_Lift, LiftConstants.kA_Lift);
    private final PIDController liftPIDController = new PIDController(LiftConstants.kP_Lift, LiftConstants.kI_Lift, LiftConstants.kD_Lift);
    private final DigitalInput limSwInput = new DigitalInput(0);
    //private final DigitalInput limSwDDDown = new DigitalInput(2);
    //private final DigitalInput limSwDDUp = new DigitalInput(4); //Channel not yet verified
    double absEncoderRaw = 0;

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Encoder Position No Offset", getLiftAbsEncoder());
        SmartDashboard.putNumber("Lift Encoder Position Offset", getLiftAbsEncoder());
        SmartDashboard.putNumber("Left Lift Motor Velocity", liftMotorLeft.get());
        SmartDashboard.putNumber("Right Lift Motor Velocity", liftMotorRight.get());
        SmartDashboard.putNumber("Telescoping Motor Encoder Position", getTeleRelEnc());
        SmartDashboard.putNumber("Telescope Motor Current", telescopeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Limit Switch State", limSwInput.get());
        //SmartDashboard.putBoolean("Limit Switch Drop Down Down", limSwDDDown.get());
        //SmartDashboard.putBoolean("Limit Switch Drop Down Up", limSwDDUp.get());


    } 

    public LiftSubsystem() {
    
            //resetLiftAbsEncoder();
        
      }

    public void setLiftMotor(double velocity) {
  
        try {

            //if(limSwDDDown.get()/*&& !limSwDDUp.get() */) { 
                //Booleans not resolved, should be when DD is in down position
                if ((MathMethods.signDouble(velocity) == -1 && getLiftAbsEncoder() < LiftConstants.kMaxLiftPosition) || 
                    (MathMethods.signDouble(velocity) == 1 && getLiftAbsEncoder() > LiftConstants.kMinLiftPosition)) {
                    liftMotorLeft.set(velocity);
                    liftMotorRight.set(-velocity);
                } else {
                    liftMotorLeft.set(0.0);
                    liftMotorRight.set(0.0);
                }
          // }/
            /*if((!limSwDDDown.get() && !limSwDDUp.get()) || (limSwDDDown.get() && limSwDDUp.get())) { 
                //Booleans not resolved, should be when DD is stuck in the middle or something fucky wucky is happening L
                if ((MathMethods.signDouble(velocity) == -1 && getLiftAbsEncoder() < LiftConstants.kMaxLiftPositionDDInbetween) || 
                    (MathMethods.signDouble(velocity) == 1 && getLiftAbsEncoder() > LiftConstants.kMinLiftPositionDDInbetween)) {
                    liftMotorLeft.set(velocity);
                    liftMotorRight.set(-velocity);
                } else {
                    liftMotorLeft.set(0.0);
                    liftMotorRight.set(0.0);
                }
            }*/
          /* 
            if(!limSwDDDown.get() && getLiftAbsEncoder() > LiftConstants.kMinLiftPositionDDUpOutside/*&& limSwDDUp.get() *///) {
                //Booleans not resolved, should be when DD is in up position
             /*    if ((MathMethods.signDouble(velocity) == -1 && getLiftAbsEncoder() < LiftConstants.kMaxLiftPositionDDUpOutside) || 
                    (MathMethods.signDouble(velocity) == 1 && getLiftAbsEncoder() > LiftConstants.kMinLiftPositionDDUpOutside)) {
                    liftMotorLeft.set(velocity);
                    liftMotorRight.set(-velocity);
                } else {
                    liftMotorLeft.set(0.0);
                    liftMotorRight.set(0.0);
                }
            }
*/ /* 
            if(!limSwDDDown.get() && getLiftAbsEncoder() < LiftConstants.kMinLiftPositionDDUpInside/*&& limSwDDUp.get() *///) {
                //Booleans not resolved, should be when DD is in up position
                /* 
                if ((MathMethods.signDouble(velocity) == -1 && getLiftAbsEncoder() < LiftConstants.kMaxLiftPositionDDUpInside) || 
                    (MathMethods.signDouble(velocity) == 1 && getLiftAbsEncoder() > LiftConstants.kMinLiftPositionDDUpInside)) {
                    liftMotorLeft.set(velocity);
                    liftMotorRight.set(-velocity);
                } else {
                    liftMotorLeft.set(0.0);
                    liftMotorRight.set(0.0);
                }
            }
 */

        } catch(Exception e) {
            System.out.println("Error: Lift Motor is Set to a value out of valid range [-1.0, 1.0]");
        }
    }

    public void setLiftMotorNoBounds(double velocity) {
            liftMotorLeft.set(velocity);
            liftMotorRight.set(-velocity);
    }

    public void stopLiftMotor() {

        liftMotorLeft.set(0);
        liftMotorRight.set(0);
    }

    public void setTelescopeMotor(double velocity) {
        //FLIP LIMIT SWITCH BACK 
        if (limSwInput.get() && velocity<0) {
            telescopeMotor.set(0);
        } else {
            telescopeMotor.set(velocity);
        }
    }

    public Boolean getLimitSwitchInput() {
        return limSwInput.get();
    }
    
    /*public Boolean getLimitSwitchDDDown() {
        return limSwDDDown.get();
    }

        
    public Boolean getLimitSwitchDDUp() {
        return limSwDDUp.get();
    }*/

    public void stopTelescopeMotor() {
        telescopeMotor.set(0);
    }



    /*public double getLiftRelEncoderRad() {
        //use this code when lift encoder is wired up
        
        double angle = liftRelEncoder.getPosition();
        angle = Math.toRadians(angle);
        return angle;
    
        //return 10;
    }*/
    /* 
    public double getLiftRelEnc() {
        double angle = Math.abs(360*liftRelEncoder.getPosition());
        return angle;
    }
    */
    public double getLiftAbsEncoder() {
        //liftAbsEncoder.setDistancePerRotation(0.15);
        liftAbsEncoder.setPositionOffset(0);
        absEncoderRaw = liftAbsEncoder.getAbsolutePosition()+LiftConstants.kLiftOffset;
        if (absEncoderRaw < LiftConstants.kLiftEncoderBreakpoint) {
            return (absEncoderRaw+1);
        } else {
            return absEncoderRaw;
        }
    }

    public void resetLiftAbsEncoder() {
        //liftAbsEncoder.reset();
    }
    
    public double getTeleRelEnc() {
        double position = 360*telescopeEnc.getPosition();
        return position;
    }


}
