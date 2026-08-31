package frc.robot.helpers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.SHOOTER;

public class TalonFXControl {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;
    private RelativeEncoder motorEncoder;

    public TalonFXControl(int canId, boolean coastMode){
        motor = new TalonFX(canId);
        motorConfig = new TalonFXConfiguration();
        if (coastMode){
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }
        else{
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        }

        motor.getConfigurator().apply(motorConfig);

        motor.set(0);

    }

    //TODO: check the setPosition method, it may not be correct. The TalonFX uses a different control mode for position control, and the PositionVoltage class may not be the right one to use. You may want to use the PositionControl class instead.
    // public void setPosition(double rotations){
    //     PositionVoltage positionVoltage = new PositionVoltage(rotations, 0);
    //     motor.setControl(positionVoltage).withPosition(rotations);
    //     motor.setPosition(rotations);
    // }

    public void setVoltage(double voltage){
        motor.setPosition(voltage);
    }

    public void setPercentOutput(double power){
        motor.set(power);
    }

    public void stop(){
        motor.set(0);
    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    public double getPosition(){
        return motorEncoder.getPosition();
    }

    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }

    public void setForwardSoftLimit(double forwardRotations){

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardRotations;
    }

    public void setReverseSoftLimit(double reverseRotations){
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseRotations;
    }

    //TODO: check the setCurrentLimit method, it may not be correct. The TalonFX uses a different control mode for current limiting, and the CurrentLimit class may not be the right one to use. You may want to use the StatorCurrentLimit class instead.
    // public void setCurrentLimit(double currentLimit){
    //     motor.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);
    //     motorConfig.withCurrentLimits(currentLimit);

    //     motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // }



    
}

