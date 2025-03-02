package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTurnAlign extends Command {
    DriveSubsystem driveSubsystem;
    PIDController pid_control = new PIDController(10/6. * 6. * 0.50,0,0);

    public AprilTurnAlign(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    //     ☆*: .｡. o(≧▽≦)o .｡.:*☆

    @Override
    public void execute() {
        final double ry = CameraSystem.ry;
        //Trying to accommodate for the value not being 0 - 1
        double pid_calc = MathUtil.clamp(pid_control.calculate(ry,0), -0.5, 0.5);

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
