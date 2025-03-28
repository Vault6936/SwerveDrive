package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.Supplier;

public class SelectTeleAuto extends Command
{
    final Supplier<RobotGoal> goalSupplier;
    SequentialCommandGroup sg;
    final ChoreoSubsystem choreo;
    public SelectTeleAuto(Supplier<RobotGoal> goal, ChoreoSubsystem chr, DriveSubsystem drive)
    {
        goalSupplier = goal;
        choreo = chr;
        addRequirements(choreo, drive);
    }

    @Override
    public void initialize()
    {
        sg = choreo.runTeleAuto(goalSupplier.get());
        sg.schedule();
    }

    @Override
    public boolean isFinished()
    {
        return sg.isFinished();
    }

    @Override
    public void end(boolean isCancelled)
    {
        sg.cancel();
    }
}
