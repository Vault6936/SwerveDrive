package frc.robot.subsystems.Lift;

public enum LiftPresets {
    BOTTOM(0),
    BOTTOM_REEF(188.6),
    MIDDLE_REEF(302),
    TOP_REEF(440),
    ALGAE_LOW(98.096),
    ALGAE_HIGH(225),
    SAFE_LOW_DRIVE(68.5);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
