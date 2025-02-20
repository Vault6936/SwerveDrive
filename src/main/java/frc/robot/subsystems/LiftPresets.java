package frc.robot.subsystems;

public enum LiftPresets {
    POSITION_1(0.0),
    POSITION_2(-35.0),
    POSITION_3(-70.0),
    POSITION_4(-105.0);

    public final double position;

    LiftPresets(double pos)
    {
        this.position = pos;
    }
}
