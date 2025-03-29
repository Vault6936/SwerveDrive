package frc.robot.subsystems.Algae;

public enum AlgaePresets {
    MINIMUM(0.0),
    MAXIMUM(.65),
    SAFE_MOVE(.0656),
    GRAB(1);
    public final double position;
    AlgaePresets(double pos)
        {
            this.position = pos;
        }
}

