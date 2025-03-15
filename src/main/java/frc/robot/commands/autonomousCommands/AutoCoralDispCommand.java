package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.MotorDirection;

import java.awt.*;

public class AutoCoralDispCommand extends Command {
    /*
    Manually moves the Coral dispensers using "While True" on a button
    */
    CoralSubsystem subsystem;
    MotorDirection direction;
    double endTime; //Seconds
    double currTime;

    public AutoCoralDispCommand(CoralSubsystem system, MotorDirection dir, double runTime /* in seconds */)
    {
        currTime = Timer.getTimestamp();
        endTime = currTime + runTime;
        subsystem = system;
        direction = dir;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute()
    {
        subsystem.setDispenser(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setDispenser(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return currTime >= endTime;
    }
}
