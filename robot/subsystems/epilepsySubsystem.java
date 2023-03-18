package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;



public class epilepsySubsystem extends SubsystemBase{
    private int firstPixelHue;
    private final AddressableLED seizureEnducer = new AddressableLED(LEDConstants.kAddressableLightsID);
    private final AddressableLEDBuffer seizLedBuffer = new AddressableLEDBuffer(LEDConstants.kAddressableLightsLength);

    public epilepsySubsystem() {
        firstPixelHue = 0;
        seizureEnducer.setLength(seizLedBuffer.getLength());
        seizureEnducer.setData(seizLedBuffer);
        seizureEnducer.start();
    }

    public void setAllColorUniform (int r, int g, int b) {
        for (var i = 0; i < seizLedBuffer.getLength(); i++) {
            seizLedBuffer.setRGB(i, r, g, b);
         }
         
         seizureEnducer.setData(seizLedBuffer);
    }

    public void setAllColorWithDelay (int r, int g, int b, int delay) {
        for (var i = 0; i < seizLedBuffer.getLength(); i++) {
            seizLedBuffer.setRGB(i, r, g, b);
            //add delay
         }
         
         seizureEnducer.setData(seizLedBuffer);
    }

  public void Rainbow() {
    // For every pixel
    for (var i = 0; i < seizLedBuffer.getLength(); i++) {
      // Set the value
      seizLedBuffer.setHSV(i, firstPixelHue+(i * 180 / seizLedBuffer.getLength()), 255, 128);

    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    

   seizureEnducer.setData(seizLedBuffer);

  }

  public void transPride() {
    int[] blueArray = {0,1,2,12,13,14,15,16,17,27,28,29};
    int[] pinkArray = {3,4,5,9,10,11,18,19,20,24,25,26};
    int[] whiteArray = {6,7,8,21,22,23};
    for (var i = 0; i < blueArray.length; i++) {
        //seizLedBuffer.setRGB(blueArray[i], 65, 70, 255);
        seizLedBuffer.setHSV(blueArray[i], 100, 255, 165);
    }
    for (var i = 0; i < pinkArray.length; i++) {
        //seizLedBuffer.setRGB(pinkArray[i], 255, 15, 135);
        seizLedBuffer.setHSV(pinkArray[i], 168, 220, 120);
    }
    for (var i = 0; i < whiteArray.length; i++) {
        //seizLedBuffer.setRGB(whiteArray[i], 255, 255, 255);
        seizLedBuffer.setHSV(whiteArray[i], 0, 0, 200);
    }

    seizureEnducer.setData(seizLedBuffer);
  }


  public void turnOff() {
    for (int i = 0; i < seizLedBuffer.getLength(); i++) {
        seizLedBuffer.setHSV(i, 0, 0, 0);
    }
    seizureEnducer.setData(seizLedBuffer);
  }

  public void moveableRainbow() {
    // For every pixel    
    for (var i = 0; i < seizLedBuffer.getLength(); i++) {
      final var hue = (0 /*change later*/+ (i * 180 / seizLedBuffer.getLength())) % 180;
      // Set the value
      seizLedBuffer.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // Check bounds
    firstPixelHue %= 180;

   seizureEnducer.setData(seizLedBuffer);

  }
    
}
