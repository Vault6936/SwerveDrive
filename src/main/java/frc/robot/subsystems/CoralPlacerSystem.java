package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.RelayJNI;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralPlacerSystem extends SubsystemBase {
    //DigitalOutput
    SparkMax intake = new SparkMax(Constants.CANIds.coral, SparkLowLevel.MotorType.kBrushless);

    public void setIntake(MotorDirection dir) {
        switch (dir) {
            case FORWARD -> intake.set(0.4);  //do we need to change the speed?
            case STOP -> intake.set(0);
            case REVERSE -> intake.set(-0.4);
        }
    }
}
