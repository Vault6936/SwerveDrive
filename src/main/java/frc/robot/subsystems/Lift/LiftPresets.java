package frc.robot.subsystems.Lift;

public enum LiftPresets {
    BOTTOM(0),
    BOTTOM_REEF(175.7),
    MIDDLE_REEF(280.891),
    TOP_REEF(438.87),
    ALGAE_LOW(116),
    ALGAE_HIGH(225),
    SAFE_LOW_DRIVE(68.5);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
