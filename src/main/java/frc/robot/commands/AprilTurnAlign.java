package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTurnAlign extends Command {
    double turn_amount;
    DriveSubsystem driveSubsystem;
    PIDController pid_control = new PIDController(0.05,0,0);

    public AprilTurnAlign(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    //     ☆*: .｡. o(≧▽≦)o .｡.:*☆

    @Override
    public void execute() {
        double ry = CameraSystem.ry;
        this.turn_amount = ry;
        //Trying to accommodate for the value not being 0 - 1
        double pid_calc = 6 * MathUtil.clamp(pid_control.calculate(ry,0), -6, 6);
        SmartDashboard.putNumber("PID VALUE: ",pid_calc);
        driveSubsystem.drive(0,0,pid_calc);

//        if (tx < -2){
//            driveSubsystem.drive(-Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else if (tx > 2) {
//            driveSubsystem.drive(Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else{
//            driveSubsystem.drive(0,0,0);
//        }
    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }


    @Override
    public boolean isFinished()
    {
        return CameraSystem.id == 0;
    }

}
