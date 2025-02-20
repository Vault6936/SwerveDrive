package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MotorDirection;
import frc.robot.subsystems.PerpendicularSensorSystem;

public class AlignPerpendicularCommand extends Command {

    PerpendicularSensorSystem perpendicularSensorSystem;
    DriveSubsystem driveSubsystem;

    public AlignPerpendicularCommand(PerpendicularSensorSystem pss, DriveSubsystem ds)
    {
        perpendicularSensorSystem = pss;
        driveSubsystem = ds;

        addRequirements(perpendicularSensorSystem);
        addRequirements(driveSubsystem);


    }


    @Override
    public void initialize()
    {
        perpendicularSensorSystem.Reset();

    }

    @Override
    public void execute()
    {
        double turn = perpendicularSensorSystem.Calculate();
        driveSubsystem.drive(0,0,turn);

    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }

    @Override
    public boolean isFinished()
    {
        return perpendicularSensorSystem.Complete();
    }

}
