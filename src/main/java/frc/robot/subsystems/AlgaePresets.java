package frc.robot.subsystems;

public enum AlgaePresets {
    DEFAULT_DOWN(0.0),
    POSITION_2(300.0);
    //POSITION_3(0.0),
    //POSITION_4(0.0);
    public final double position;
    AlgaePresets(double pos)
        {
            this.position = pos;
        }
}

