package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftPresets;
import frc.robot.subsystems.LiftSystem;

public class LiftPresetCommand  extends Command {

    private final LiftSystem liftSystem;
    LiftPresets target_preset;

    public LiftPresetCommand(LiftSystem liftSystem, LiftPresets preset)
    {

        this.liftSystem = liftSystem;
        target_preset = preset;
    }

    @Override
    public void initialize() {
        liftSystem.goPreset(target_preset);
    }


    // Called every time the scheduler runs while the command is scheduled.
//    @Override
//    public void execute() {
//    }

//    @Override
//    public boolean isFinished()
//    {
//        return finished;
//    }

    @Override
    public void end(boolean cancelled)
    {
        liftSystem.stop();
    }
}
