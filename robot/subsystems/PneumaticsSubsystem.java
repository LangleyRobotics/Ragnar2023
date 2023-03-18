package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PneumaticsConstants;



public class PneumaticsSubsystem extends SubsystemBase{
    private final Solenoid sully = new Solenoid(PneumaticsModuleType.REVPH, PneumaticsConstants.kSullySolenoid);
    private final Solenoid sally = new Solenoid(PneumaticsModuleType.REVPH, PneumaticsConstants.kSallySolenoid);

    private final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sully?", sully.get());
        SmartDashboard.putBoolean("Sally", sally.get());
        compressor.enableDigital();
    }

    public void toggleSully() {
        sully.toggle();
    }

    public boolean isSullyAwake() {
        return sully.get();
    }

    public int wheresSully() {
        return sully.getChannel();
    }

    public void toggleSally() {
        sally.toggle();
    }

    public boolean isSallyAwake() {
        return sally.get();
    }

    public int wheresSally() {
        return sally.getChannel();
    }

}

    

