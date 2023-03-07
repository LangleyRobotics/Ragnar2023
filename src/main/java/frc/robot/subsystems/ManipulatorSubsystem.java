package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax clawMotor = new CANSparkMax(ManipulatorConstants.kClawMotor, MotorType.kBrushed);
    private final CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.kIntakeMotor, MotorType.kBrushless);

    @Override
    public void periodic() {

    }

    public void setClawMotor(double velocity) {
        clawMotor.set(velocity);
    }

    public void stopClawMotor(){
        clawMotor.set(0);
    }

    public void setIntakeMotor(double velocity){
        intakeMotor.set(velocity);
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }

}
