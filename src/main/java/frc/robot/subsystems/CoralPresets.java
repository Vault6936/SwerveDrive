package frc.robot.subsystems;

public enum CoralPresets {
    POSITION_1(0.0),
    POSITION_2(300.0);
    //POSITION_3(0.0),
    //POSITION_4(0.0);
    public final double position;
    CoralPresets(double pos)
    {
        this.position = pos;
    }
}
