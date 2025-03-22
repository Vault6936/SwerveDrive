package frc.robot.subsystems;

public enum LiftPresets {
    BOTTOM(0),
    BOTTOM_REEF(87),
    MIDDLE_REEF(204),
    TOP_REEF(370),
    SAFE_LOW_DRIVE(68.5);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
