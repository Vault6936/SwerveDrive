package frc.robot.subsystems;

public enum LiftPresets {
    STARTING(0.0),
    TROUGH(-174),
    HEIGHT_1(-209),
    HEIGHT_2(-329);
    //HEIGHT_3(-470);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
