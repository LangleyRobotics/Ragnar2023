package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;


public class LimelightSubsystem extends SubsystemBase{
    private static LimelightSubsystem instance;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry tv;
    private NetworkTableEntry ta;
    private NetworkTableEntry camMode;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry pipeline;

    private NetworkTable tableLow;
    private NetworkTableEntry txLow;
    private NetworkTableEntry tyLow;
    private NetworkTableEntry tvLow;
    private NetworkTableEntry taLow;
    private NetworkTableEntry camModeLow;
    private NetworkTableEntry ledModeLow;
    private NetworkTableEntry pipelineLow;
  

    public static LimelightSubsystem getInstance()
    {
      if (instance == null)
        instance = new LimelightSubsystem();
      
      return instance;
    }
  
    /**
     * Get limelight data from network table.
     */
    public LimelightSubsystem()
    {
      //CameraServer.startAutomaticCapture();
      table = NetworkTableInstance.getDefault().getTable("limelight-high");
      tx = table.getEntry("tx"); 
      ty = table.getEntry("ty"); 
      tv = table.getEntry("tv"); 
      ta = table.getEntry("ta");
      ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
      camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).
      pipeline = table.getEntry("pipeline");

      //low limelight, rename in limelight online interface
      tableLow = NetworkTableInstance.getDefault().getTable("limelight-low");
      txLow = tableLow.getEntry("tx"); 
      tyLow = tableLow.getEntry("ty"); 
      tvLow = tableLow.getEntry("tv"); 
      taLow = tableLow.getEntry("ta");
      ledModeLow = tableLow.getEntry("ledMode"); // limelight's LED state (0-3).
      camModeLow = tableLow.getEntry("camMode"); // limelight's operation mode (0-1).
      pipelineLow = tableLow.getEntry("pipeline");
    }
    
    @Override
    public void periodic() {
        //read values periodically
        double x = getTargetOffsetX();
        double y = getTargetOffsetY();
        double area = getTargetArea();
        boolean targetInView = targetInView();

        double xLow = getTargetOffsetXLow();
        double yLow = getTargetOffsetYLow();
        double areaLow = getTargetAreaLow();
        boolean targetInViewLow = targetInViewLow();

        double[] LL_botpose = NetworkTableInstance.getDefault().getTable("limeligh-low").getEntry("botpose").getDoubleArray(new double[6]);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("Limelight Target in view", targetInView);
        SmartDashboard.putBoolean("Driver Mode", isDriverMode());
        SmartDashboard.putBoolean("Vision Mode ", isVisionMode());


        SmartDashboard.putNumber("LimelightX Low", xLow);
        SmartDashboard.putNumber("LimelightY Low", yLow);
        SmartDashboard.putNumber("LimelightArea Low", areaLow);
        SmartDashboard.putBoolean("Limelight Target in view Low", targetInViewLow);
        //put botpose values in smart dashboard
        try {
          SmartDashboard.putNumber(" Translation BotPose X", LL_botpose[0]);
          SmartDashboard.putNumber("Translation BotPose Y", LL_botpose[1]);
          SmartDashboard.putNumber("Translation BotPose Z", LL_botpose[2]);
          SmartDashboard.putNumber("Rotation BotPose Roll", LL_botpose[3]);
          SmartDashboard.putNumber("Rotation BotPose Pitch", LL_botpose[4]);
          SmartDashboard.putNumber("Rotation BotPose Yaw", LL_botpose[5]);
        } catch(Exception e) {
          SmartDashboard.putNumber(" Translation BotPose X", 0);
          SmartDashboard.putNumber("Translation BotPose Y", 0);
          SmartDashboard.putNumber("Translation BotPose Z", 0);
          SmartDashboard.putNumber("Rotation BotPose Roll", 0);
          SmartDashboard.putNumber("Rotation BotPose Pitch", 0);
          SmartDashboard.putNumber("Rotation BotPose Yaw", 0);
        }

    }
    
    

    public double getTargetOffsetX()
    {
      return tx.getDouble(0.0);
    }
  
    public double getTargetOffsetY()
    {
      return ty.getDouble(0.0);
    }

    public boolean targetInView()
    {
        if (tv.getNumber(0).intValue() == 1) {
            return true;
        }
        return false;
    }
  
    public double getTargetArea()
    {
      return ta.getDouble(0.0);
    }
  
    private void setLEDMode(int mode)
    {
      ledMode.setNumber(mode);
    }
    
    public void setPipeline(int id) {
      pipeline.setNumber(id);
    }

    public long getPipeline() {
      return pipeline.getInteger(0);
    }

    public void turnOnLED()
    {
      this.setLEDMode(LimelightConstants.kledModeOn);
    }                 
    public void turnOffLED()
    {
      this.setLEDMode(LimelightConstants.kledModeOff);
    }
    public void blinkLED()
    {
      this.setLEDMode(LimelightConstants.kledModeBlink);
    }

  
    private void setCamMode(int mode)
    {
      camMode.setNumber(mode);
    }
  
    public void setModeDriver()
    {
      this.setLEDMode(LimelightConstants.kledModeOff);
      this.setCamMode(LimelightConstants.kcamModeDriverCamera);

    }
  
    public void setModeVision()
    {
      this.setLEDMode(LimelightConstants.kledModeOn);
      this.setCamMode(LimelightConstants.kcamModeVisionProcessor);
    }
  

    private boolean isDriverMode()
    {
      return ledMode.getDouble(0.0) == LimelightConstants.kledModeOff && camMode.getDouble(0.0) == LimelightConstants.kcamModeDriverCamera;
    }
    private boolean isVisionMode()
    {
      return ledMode.getDouble(0.0) == LimelightConstants.kledModeOn && camMode.getDouble(0.0) == LimelightConstants.kcamModeVisionProcessor;
    }
    
    public void toggleMode()
    {
      if (this.isDriverMode())
      {
        this.setModeVision();
      }
      else if (this.isVisionMode())
      {
        this.setModeDriver();
      }
      else
      {
        this.blinkLED();
      }

    }

    public double getTargetOffsetXLow()
    {
      return txLow.getDouble(0.0);
    }
  
    public double getTargetOffsetYLow()
    {
      return tyLow.getDouble(0.0);
    }

    public boolean targetInViewLow()
    {
        if (tvLow.getNumber(0).intValue() == 1) {
            return true;
        }
        return false;
    }
  
    public double getTargetAreaLow()
    {
      return taLow.getDouble(0.0);
    }
  
    private void setLEDModeLow(int mode)
    {
      ledModeLow.setNumber(mode);
    }
    
    public void setPipelineLow(int id) {
      pipelineLow.setNumber(id);
    }

    public static double[] get_LL_botpose()
    {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    }
  
  }
