package frc.robot.subsystems;

//import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;



public class lightingSubsystem extends SubsystemBase{
    private int firstPixelHue;
    private final AddressableLED seizureEnducer = new AddressableLED(LEDConstants.kAddressableLightsID);
    private final AddressableLEDBuffer seizLedBuffer = new AddressableLEDBuffer(LEDConstants.kAddressableLightsLength);


    //private HashMap<Character, int[]> lettersToLights = new HashMap<Character, int[]>();



    public lightingSubsystem() {
        firstPixelHue = 0;
        seizureEnducer.setLength(seizLedBuffer.getLength());
        seizureEnducer.setData(seizLedBuffer);
        seizureEnducer.start();
    }

    public void mapLetters() {
      //lettersToLights.get('f');
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
    for (var i = 0; i < seizLedBuffer.getLength(); i++) {
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
        seizLedBuffer.setHSV(blueArray[i], 100, 255, 165);
    }
    for (var i = 0; i < pinkArray.length; i++) {
        seizLedBuffer.setHSV(pinkArray[i], 168, 220, 120);
    }
    for (var i = 0; i < whiteArray.length; i++) {
        seizLedBuffer.setHSV(whiteArray[i], 0, 0, 200);
    }

    seizureEnducer.setData(seizLedBuffer);
  }

  public void biPride() {
    int[] blueArray = {0,1,2,3,4,5,15,16,17,18,19,20};
    int[] pinkArray = {9,10,11,12,13,14,24,25,26,27,28,29};
    int[] purpleArray = {6,7,8,21,22,23};
    for (var i = 0; i < blueArray.length; i++) {
        seizLedBuffer.setHSV(blueArray[i], 100, 255, 165);
    }
    for (var i = 0; i < pinkArray.length; i++) {
        seizLedBuffer.setHSV(pinkArray[i], 168, 220, 120);
    }
    for (var i = 0; i < purpleArray.length; i++) {
        seizLedBuffer.setRGB(purpleArray[i], 75,0, 120);
    }

    seizureEnducer.setData(seizLedBuffer);
  }


  public void turnOff() {
    for (int i = 0; i < seizLedBuffer.getLength(); i++) {
        seizLedBuffer.setHSV(i, 0, 0, 0);
    }
    seizureEnducer.setData(seizLedBuffer);
  }

  public void rainbowFromCenter() {
    for (var i = 0; i < seizLedBuffer.getLength()/2; i++) {
        seizLedBuffer.setHSV(i, firstPixelHue+(i * 180 / seizLedBuffer.getLength()), 255, 128);
        seizLedBuffer.setHSV(seizLedBuffer.getLength()- i-1, firstPixelHue+(i * 180 / seizLedBuffer.getLength()), 255, 128);
    }
      // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    seizureEnducer.setData(seizLedBuffer);
  }


    
}
