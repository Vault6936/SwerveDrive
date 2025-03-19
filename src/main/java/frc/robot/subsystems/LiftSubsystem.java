package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class LiftSubsystem extends SubsystemBase {

    TalonFX extendLeft =  new TalonFX(Constants.CANIds.leftLift);
    TalonFX extendRight =  new TalonFX(Constants.CANIds.rightLift);
    PIDController pid = new PIDController(0.05, 0, 0);

    double currentTargetPos;
    DoubleSupplier encoderLeft;
    DoubleSupplier encoderRight;

    final double CHANGE_MULTIPLIER = 2.2;
    final double MAX_SPEED_PERCENT = .95;

    static final double min_position = 0.0;
    static final double max_position = 370;

    DoubleConsumer setSpeedMultiplier;

    CoralSubsystem coralSubsystem;
    AlgaeSubsystem algaeSubsystem;
    boolean canLowerFully;


    public LiftSubsystem(DoubleConsumer setSpeedMultiplier, CoralSubsystem coralSystem, AlgaeSubsystem algaeSystem)
    {
        pid.setTolerance(10);
        extendLeft.setPosition(0);
        extendRight.setPosition(0);
        extendLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(10));
        extendLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(10));
        encoderLeft = () -> extendLeft.getPosition().getValueAsDouble();
        encoderRight = () -> extendRight.getPosition().getValueAsDouble();
        currentTargetPos = 0;
        coralSubsystem = coralSystem;
        algaeSubsystem = algaeSystem;
        this.setSpeedMultiplier = setSpeedMultiplier;
    }

    /***
     * Used to alter the speed of the robot based on the lift's vertical position based on a logistic function
     */
    public double getDriveSpeedMultiplier(){
        return .8 / (1 + Math.pow(Math.E,.457799 * (getCurrentPosition() - 237.6))) + .2;
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
        currentTargetPos = getCurrentPosition();
    }

    public double getCurrentPosition()
    {
        return (encoderLeft.getAsDouble() - encoderRight.getAsDouble()) / 2;
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
                extendLeft.set(0);
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
        double outputPower = pid.calculate(getCurrentPosition(), currentTargetPos) * Constants.SpeedConstants.LIFT_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT) * Constants.REMOVE_THIS_CLASS_PLEASE.slowDriveMultiplier;

        if(Math.abs(encoderRight.getAsDouble()) > 400 || Math.abs(encoderLeft.getAsDouble()) > 400)
        {
            extendRight.set(0);
            extendLeft.set(0);
            return;
        }

        // If the (lift is low enough) and (algaeAngle and coralHoz will collide with something)
        if (currentTargetPos > -150 && !canLowerFully) {
            move_components_safe_pos();
        }


        else {
            extendLeft.set(outputPower);
            extendRight.set(-outputPower);
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
        setSpeedMultiplier.accept(getDriveSpeedMultiplier());
        if (Constants.DebugInfo.debugLift) {
            SmartDashboard.putNumber("Lift speed multiplier", getDriveSpeedMultiplier());
        }
        SmartDashboard.putNumber("LiftPosition", getCurrentPosition());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
    }
}
