package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class LedSubsystem {
    SerialPort serial = new SerialPort(9600, SerialPort.Port.kMXP);

    public LedSubsystem()
    {
        serial.writeString("B2");
    }

    //All parameters should be inputted as single characters
    public void TeamColor(String teamColor, String intensity)
    {
        serial.writeString(teamColor+intensity);
    }

    public void ReadyToLoad(String intensity)
    {
        serial.writeString("Y"+intensity);
    }

    public void IsLoaded(String intensity)
    {
        serial.writeString("G"+intensity);
    }

}
