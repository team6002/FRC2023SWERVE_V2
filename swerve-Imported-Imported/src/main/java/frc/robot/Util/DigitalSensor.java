package frc.robot.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DigitalSensor extends Button{
    DigitalInput sensor;

    public DigitalSensor(int portNum)
    {
        sensor = new DigitalInput(portNum);
    }
    
    @Override
    public boolean get()
    {
        return sensor.get();
    }

    public void and(boolean b) {
    }
}