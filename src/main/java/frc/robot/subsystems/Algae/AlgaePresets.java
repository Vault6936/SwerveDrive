package frc.robot.subsystems.Algae;

public enum AlgaePresets {
    MINIMUM(0.0),
    MAXIMUM(300.0),
    SAVE_MOVE(313232),
    GRAB(1);
    public final double position;
    AlgaePresets(double pos)
        {
            this.position = pos;
        }
}

