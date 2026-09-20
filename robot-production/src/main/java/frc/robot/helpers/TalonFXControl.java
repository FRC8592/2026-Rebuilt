package frc.robot.helpers;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.INTAKE;
import com.ctre.phoenix6.controls.*;

public class TalonFXControl {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;
    private RelativeEncoder motorEncoder;
    private CurrentLimitsConfigs motorCurrentLimit;
    private int CAN_ID;
    private boolean coastMode;

    public TalonFXControl(int canId, boolean coastMode){
        motor = new TalonFX(canId);
        CAN_ID = canId;
        motorConfig = new TalonFXConfiguration();
        if (coastMode){
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }
        else{
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        }

        this.coastMode = coastMode;

        motor.getConfigurator().apply(motorConfig);

        motorCurrentLimit = new CurrentLimitsConfigs();

        motor.set(0);

    }
    public void setPosition(double rotations){
        PositionVoltage positionVoltage = new PositionVoltage(rotations);
        motor.setControl(positionVoltage.withPosition(rotations));
        motor.setPosition(rotations);
    }

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

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getPosition(){
        return motorEncoder.getPosition();
    }

    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }

    public void setFollower(TalonFXControl followerMotor, MotorAlignmentValue alignmentValue){
        motor.setControl(new Follower(followerMotor.CAN_ID, alignmentValue));
    }

    public void setForwardSoftLimit(double forwardRotations){

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardRotations;
    }

    public void setReverseSoftLimit(double reverseRotations){
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseRotations;
    }

    public void setCurrentLimit(double currentLimit){
        motorCurrentLimit.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);
        motorConfig.withCurrentLimits(motorCurrentLimit);
    }

    // public void logMotorValues(String logPath, String motorName){
    //     Logger.recordOutput(logPath + , getIntakeVelocity() * 60d);
    // }
}

