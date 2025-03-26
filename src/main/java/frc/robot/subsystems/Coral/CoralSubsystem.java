package frc.robot.subsystems.Coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Other.MotorDirection;

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

    PIDController pidHoz = new PIDController(0.03, 0, 0); //TODO SET P VALUE CORRECTLY
    boolean leftSwitch;
    boolean rightSwitch;
    boolean centerSwitch;

    static final double maxPos = 24.7;
    static final double minPos = -24.7;

    public CoralSubsystem(){
        coralHoz.getAnalog();
        rightSwitch = coralHoz.getForwardLimitSwitch().isPressed();
        leftSwitch = coralHoz.getReverseLimitSwitch().isPressed();
        coralDispenser1.getForwardLimitSwitch();
        hozEncoder = coralHoz.getEncoder();
        hozEncoder.setPosition(0);
        ((SparkMax) coralHoz).configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
        hozTargetPos = MathUtil.clamp(hozTargetPos + (change * 1.0), minPos, maxPos);
    }

    public void slideToPreset(CoralPresets presets){
        hozTargetPos = presets.position;
    }

    public void stopMoveToPos(){
        hozTargetPos = hozEncoder.getPosition();
    }

    public void doPositionControl() {
        double outputPower = pidHoz.calculate(hozEncoder.getPosition(), hozTargetPos) * Constants.SpeedConstants.CORAL_HOZ_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1) *Constants.REMOVE_THIS_CLASS_PLEASE.slowDriveMultiplier;
        coralHoz.set(outputPower);
        if (Constants.DebugInfo.debugCoral){
            SmartDashboard.putNumber("Coral Horizontal Power", outputPower);
        }
    }


    public boolean getGateBool(){
        return coralDispenser1.getForwardLimitSwitch().isPressed();
    }

    @Override
    public void periodic()
    {
        rightSwitch = coralHoz.getForwardLimitSwitch().isPressed();
        leftSwitch = coralHoz.getReverseLimitSwitch().isPressed();
        centerSwitch = (coralHoz.getAnalog().getVoltage() < 1);
        doPositionControl();
        if (centerSwitch){
            hozEncoder.setPosition(0);
        }
        if (leftSwitch){
            hozEncoder.setPosition(minPos);
        }
        if (rightSwitch){
            hozEncoder.setPosition(maxPos);
        }
        if (Constants.DebugInfo.debugCoral){
            SmartDashboard.putBoolean("Coral Left Switch", leftSwitch);
            SmartDashboard.putBoolean("Coral Right Switch", rightSwitch);
            SmartDashboard.putBoolean("Coral Center Switch", centerSwitch);

            SmartDashboard.putBoolean("Coral Gateway Triggered", getGateBool());
            SmartDashboard.putNumber("Coral Horizontal Position", hozEncoder.getPosition());
            SmartDashboard.putNumber("Coral Horizontal Target Position", hozTargetPos);
        }
    }
}
