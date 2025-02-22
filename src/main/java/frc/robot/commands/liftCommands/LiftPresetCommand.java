package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftPresets;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPresetCommand  extends Command {

    private final LiftSubsystem liftSubsystem;
    LiftPresets target_preset;

    public LiftPresetCommand(LiftSubsystem liftSubsystem, LiftPresets preset)
    {
        this.liftSubsystem = liftSubsystem;
        target_preset = preset;
    }

    @Override
    public void initialize() {
        liftSubsystem.goPreset(target_preset);
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
        liftSubsystem.stopMoveToPos();
    }
}
