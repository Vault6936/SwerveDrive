package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class LiftSubsystem extends SubsystemBase {

    TalonFX extend =  new TalonFX(Constants.CANIds.lift);
    PIDController pid = new PIDController(0.25, 0, 0);

    double currentTargetPos = 0;
    DoubleSupplier encoder_value;


    final double CHANGE_MULTIPLIER = 2.2;
    final double MAX_SPEED_PERCENT = .6;

    static final double min_position = -480.0;
    static final double max_position = 0.0;

    DoubleConsumer setSpeedMultiplier;

    CoralSubsystem coralSubsystem;
    AlgaeSubsystem algaeSubsystem;
    boolean canLowerFully;


    public LiftSubsystem(DoubleConsumer setSpeedMultiplier, CoralSubsystem coralSystem, AlgaeSubsystem algaeSystem)
    {
        extend.setPosition(0);
        encoder_value = () -> extend.getPosition().getValueAsDouble();
        currentTargetPos = 0;
        coralSubsystem = coralSystem;
        algaeSubsystem = algaeSystem;
    }

    /***
     * Used to alter the speed of the robot based on the lift's vertical position based on a logistic function
     */
    public double getDriveSpeedMultiplier(){ //TODO Alter this function, the max has been updated; but it should still work
        return .8 / (1 + 2 * Math.pow(Math.E,.02 * (encoder_value.getAsDouble() - 2./3. * max_position))) + .2;
    }

    /***
     * Updates current target position at the speed of CHANGE_MULTIPLIER
     * Input of overridden allows min and max positions to be ignored
     */
    public void updatePos(double change, boolean overridden)
    {
        if(Math.abs(change) > 0.1)
        {
            currentTargetPos = overridden ?  currentTargetPos + change * CHANGE_MULTIPLIER:
                    MathUtil.clamp(currentTargetPos + change * CHANGE_MULTIPLIER, min_position, max_position);
        }
    }

    /***
     * Target position goes to a LiftPreset preset
     */
    public void goPreset(LiftPresets preset) {
        currentTargetPos = preset.position;
    }

    /***
     * Sets currentTargetPos to current encoder value
     * DOES NOT RESET ENCODER VALUE
     */
    public void stopMoveToPos() {
        currentTargetPos = encoder_value.getAsDouble();
    }

    /***
     * Used to set the direction of the motors
     * Only in use to stop the motors.
     */
    public void setExtend(MotorDirection dir) {
        stopMoveToPos();
        switch (dir) {
            case STOP :
            {
                extend.set(0);
            }
            //If we need them:
            case FORWARD:{}
            case REVERSE:{}
        }
    }

    /***
     * Calculates output power using the pid controller.
     * if the lift is too low, moves coralHoz and algaeAngle to safe positions first.
     */
    public void doPositionControl(){
        double outputPower = pid.calculate(encoder_value.getAsDouble(), currentTargetPos) * Constants.SpeedConstants.LIFT_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

        // If the (lift is low enough) and (algaeAngle and coralHoz will collide with something)
        if (currentTargetPos > -150 && !canLowerFully) {
            move_components_safe_pos();
        }
        else {
            extend.set(outputPower);
        }
    }

    /***
     * Moves algaeAngle and coralHoz into safe place so the lift can lower.
     */
    private void move_components_safe_pos(){
        coralSubsystem.setSafePos();
        algaeSubsystem.setSafePos();
    }


    @Override
    public void periodic()
    {
        canLowerFully = coralSubsystem.isSafeToLower && algaeSubsystem.isSafeToLower;
        SmartDashboard.putNumber("LiftPosition", encoder_value.getAsDouble());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
    }
}
