// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveCalibrateCommand;
import frc.robot.commands.autonomousCommands.ToggleStop;
import frc.robot.webdashboard.WebdashboardServer;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    public static RobotContainer robotContainer;

    private Command[] initializationCommands;

    String[] reefLocs = {"ReefNW", "ReefNE", "ReefE", "ReefSE", "ReefSW", "ReefW"};
    String[] sourceLocs = {"SourceN", "SourceS"};


    /**
     * This command loads a bunch of choreo commands into memory.
     * This makes further access much faster (2.5s -> 0.05s for n=48), but risks OOM behavior.
     * See the FRC Java Garbage Collection tutorial for information on how to handle that problem.
     * In our case, we've set up a USB drive with 200MB swap file.
     * @return
     */
    private String[] preloadChoreoCommands(){
        Timer t = new Timer();
        t.start();
        double startTime = t.get();
        String[] allPaths = new String[48];
        int i = 0;
        for (String reefLoc : reefLocs){
            for (String sourceLoc : sourceLocs){
                allPaths[i] = reefLoc + sourceLoc;
                i++;
                allPaths[i] = sourceLoc + reefLoc;
                i++;
                robotContainer.choreo.selectTrajectory(reefLoc, sourceLoc);
                robotContainer.choreo.selectTrajectory(sourceLoc, reefLoc);
            }
        }
        double endTime = t.get();
        System.out.println("ReadPoseTime" + (startTime - endTime));
        return allPaths;
    }
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        robotContainer.driveSubsystem.poseReset(new Pose2d(0,0,new Rotation2d()));

        initializationCommands = new Command[]{new SwerveCalibrateCommand()};

        for (Command command : initializationCommands) {
            CommandScheduler.getInstance().schedule(command);
        }
        preloadChoreoCommands();
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putString("Current Pose:", robotContainer.driveSubsystem.currentPose.toString());
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.driveSubsystem.poseReset(new Pose2d(0,0,new Rotation2d()));


        robotContainer.lift.stopMoveToPos();
        robotContainer.algaeSubsystem.setAngle(0);
        Pose2d startTele = new Pose2d(
                robotContainer.driveSubsystem.currentPose.getX(),
                robotContainer.driveSubsystem.currentPose.getY(),
                Rotation2d.fromDegrees(0));
        robotContainer.driveSubsystem.poseReset(startTele);
        CommandScheduler.getInstance().schedule(new ToggleStop(robotContainer.driveSubsystem, false));

        //DriveSubsystem.getInstance().zeroNavX();
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }
}
