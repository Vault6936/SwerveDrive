package frc.robot.subsystems;

public enum AlgaePresets {
    DEFAULT_DOWN(0.0),
    POSITION_2(300.0),
    POSITION_3(1),
    POSITION_4(4);
    public final double position;
    AlgaePresets(double pos)
        {
            this.position = pos;
        }
}

