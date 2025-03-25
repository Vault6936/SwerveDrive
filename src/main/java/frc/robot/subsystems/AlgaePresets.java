package frc.robot.subsystems;

public enum AlgaePresets {
    MINIMUM(0.0),
    MAXIMUM(300.0),
    GRAB(1);
    public final double position;
    AlgaePresets(double pos)
        {
            this.position = pos;
        }
}

