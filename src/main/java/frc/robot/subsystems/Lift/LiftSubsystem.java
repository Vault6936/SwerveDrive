package frc.robot.subsystems.Lift;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

import java.util.Objects;
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

    static final double min_position = 0.0;
    static final double max_position = 440;

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
        double speedMult = 1 / (1 + Math.pow(Math.E, .0107 * (getCurrentPosition() - 114.363))) + .2;
        SmartDashboard.putNumber("LiftSpeedMultiplier", speedMult);
        return speedMult;
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
        if (Objects.requireNonNull(dir) == MotorDirection.STOP) {
            extendLeft.set(0);
        }
    }

    /***
     * Calculates output power using the pid controller.
     * if the lift is too low, moves coralHoz and algaeAngle to safe positions first.
     */
    public void doPositionControl(){
        double outputPower = pid.calculate(getCurrentPosition(), currentTargetPos) * Constants.SpeedConstants.LIFT_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -Constants.SpeedConstants.LIFT_SPEED, Constants.SpeedConstants.LIFT_SPEED) * Constants.REMOVE_THIS_CLASS_PLEASE.slowDriveMultiplier;

        extendLeft.set(outputPower);
        extendRight.set(-outputPower);
    }

    /***
     * Moves algaeAngle and coralHoz into safe place so the lift can lower.
     */


    @Override
    public void periodic()
    {
        setSpeedMultiplier.accept(getDriveSpeedMultiplier());
        if (Constants.DebugInfo.debugLift) {
            SmartDashboard.putNumber("Lift speed multiplier", getDriveSpeedMultiplier());
        }

        SmartDashboard.putNumber("LiftPosition", getCurrentPosition());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
    }
}
