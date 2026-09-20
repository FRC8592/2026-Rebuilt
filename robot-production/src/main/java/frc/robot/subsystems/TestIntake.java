package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestIntake extends SubsystemBase {
    private SparkMax sparkMotor = new SparkMax(0, MotorType.kBrushless);
    public TestIntake() {
    sparkMotor.set(0.5);
    sparkMotor.stopMotor();
    }
}