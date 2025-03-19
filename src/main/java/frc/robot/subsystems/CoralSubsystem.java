package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This system handles the motors for the coral placer system.
 */
public class CoralSubsystem extends SubsystemBase {
    /*
    Move Coral dispenser's coral pushers Forward/Backward
    Move Coral dispenser Left/Right (uses target pos, uses presets)
     */
    final SparkMax coralHoz = new SparkMax(Constants.CANIds.coralHoz, SparkLowLevel.MotorType.kBrushless);
    final SparkMax coralDispenser1 = new SparkMax(Constants.CANIds.coralDispenserSingleWheel, SparkLowLevel.MotorType.kBrushless);
    final SparkMax coralDispenser2 = new SparkMax(Constants.CANIds.coralDispenserDoubleWheel, SparkLowLevel.MotorType.kBrushless);
    public final RelativeEncoder hozEncoder;
    double hozTargetPos;

    PIDController pid = new PIDController(0.03, 0, 0); //TODO SET P VALUE CORRECTLY

    static final double maxPosition = 38; // 42 is the measured max, we're adding a temporary 4-point safety margin
    static final double minPosition = -44; // 48 is the measured min  //TODO SET THIS VALUE CORRECTLY

    boolean isSafeToLower;

    public CoralSubsystem(){
        coralDispenser1.getForwardLimitSwitch();
        hozEncoder = coralHoz.getEncoder();
        hozEncoder.setPosition(0);
    }

    /**
     * This moves the dispenser motors forward or backward for the coral placer.
     * @param dir Motor movement, where forward places the output.
     */
    public void setDispenser(MotorDirection dir) {
        switch (dir) {
            case FORWARD -> {
                coralDispenser1.set(Constants.SpeedConstants.CORAL_DISPENSER_SPEED);
                coralDispenser2.set(-Constants.SpeedConstants.CORAL_DISPENSER_SPEED);
            }
            case STOP -> {
                coralDispenser1.set(0);
                coralDispenser2.set(0);
            }
            case REVERSE -> {
                coralDispenser1.set(-Constants.SpeedConstants.CORAL_DISPENSER_SPEED);
                coralDispenser2.set(Constants.SpeedConstants.CORAL_DISPENSER_SPEED);
            }
        }
    }

    public void setHozCoral(MotorDirection dir) {
        stopMoveToPos();
        switch (dir) {
            case FORWARD -> coralHoz.set(Constants.SpeedConstants.CORAL_HOZ_SPEED);
            case STOP -> coralHoz.set(0);
            case REVERSE -> coralHoz.set(-Constants.SpeedConstants.CORAL_HOZ_SPEED);
        }
    }

    public void updateHozTarget(double change){
        hozTargetPos = MathUtil.clamp(hozTargetPos + (change * 1.0), minPosition, maxPosition);
    }

    public void slideToPreset(CoralPresets presets){
        hozTargetPos = presets.position;
    }

    public void stopMoveToPos(){
        hozTargetPos = hozEncoder.getPosition();
    }

    public void doPositionControl() {
        double outputPower = pid.calculate(hozEncoder.getPosition(), hozTargetPos) * Constants.SpeedConstants.CORAL_HOZ_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1) *Constants.REMOVE_THIS_CLASS_PLEASE.slowDriveMultiplier;
        coralHoz.set(outputPower);
        if (Constants.DebugInfo.debugCoral){
            SmartDashboard.putNumber("Coral Horizontal Power", outputPower);
        }
    }

    public void setSafePos(){
        if ((Math.abs(hozEncoder.getPosition() - hozTargetPos) < 10) &&
                (Math.abs(hozTargetPos - (minPosition + maxPosition) / 2.) < 10 ))  //TODO SET TOLERANCE
        {
            isSafeToLower = true;
        } else {
            isSafeToLower = false;
            slideToPreset(CoralPresets.CENTER_POSITION);
        }
    }

    public boolean getGateBool(){
        return coralDispenser1.getForwardLimitSwitch().isPressed();
    }

    @Override
    public void periodic()
    {
        if (Constants.DebugInfo.debugCoral){
            SmartDashboard.putBoolean("Coral Gateway Triggered", getGateBool());
            SmartDashboard.putNumber("Coral Horizontal Position", hozEncoder.getPosition());
            SmartDashboard.putNumber("Coral Horizontal Target Position", hozTargetPos);
        }
    }
}
