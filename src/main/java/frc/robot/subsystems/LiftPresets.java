package frc.robot.subsystems;

public enum LiftPresets {
    POSITION_0(0.0),
    POSITION_1(-174),
    POSITION_2(-209),
    POSITION_3(-329);
    //POSITION_4(-470);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
