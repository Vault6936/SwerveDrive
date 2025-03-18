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
    double runTime;

    public AutoCoralDispCommand(CoralSubsystem system, MotorDirection dir, double runTime /* in seconds */)
    {
        subsystem = system;
        direction = dir;
        this.runTime = runTime;
    }

    @Override
    public void initialize(){
        currTime = Timer.getTimestamp();
        endTime = currTime + runTime;
    }

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
        return Timer.getTimestamp() >= endTime;
    }
}
