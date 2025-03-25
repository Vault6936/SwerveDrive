package frc.robot.subsystems.Coral;

public enum CoralPresets {
    CENTER_POS(0),
    LEFT_POS(CoralSubsystem.minPos),
    RIGHT_POS(CoralSubsystem.maxPos);
    public final double position;
    CoralPresets(double pos)
    {
        this.position = pos;
    }
}
