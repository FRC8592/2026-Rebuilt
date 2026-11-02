package frc.robot.helpers;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.TURRET;

import com.ctre.phoenix6.controls.*;

public class TalonFXControl {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;
    private CurrentLimitsConfigs motorCurrentLimit;
    private Slot0Configs motorPIDSlot0Configs;
    private Slot1Configs motorPIDSlot1Configs;
    private Slot2Configs motorPIDSlot2Configs;
    private int CAN_ID;
    private VelocityVoltage velocityVoltage;
    private PositionVoltage positionRequest;

    /**
     * @param canId
     * @param coastMode
     * Constructor for TalonFX motors
     * Instantiates motor config, encoder, current limit, velocity voltage.
     */
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

        motor.getConfigurator().apply(motorConfig);

        motorCurrentLimit = new CurrentLimitsConfigs();

        motor.set(0);

        velocityVoltage = new VelocityVoltage(0);
        positionRequest = new PositionVoltage(0);
    }

    /**
     * Sets motor position using rotations
     * @param rotations
     */

    public void setPosition(double rotations){
        motor.setControl(positionRequest.withPosition(rotations));
        motor.setPosition(rotations);
    }

    public void setPositionSlot(int slot){
        if (slot == 0){
            motor.setControl(positionRequest.withSlot(0));
        }
        else if (slot == 1){
            motor.setControl(positionRequest.withSlot(1));
        }
        else if (slot == 2){
            motor.setControl(positionRequest.withSlot(2));
        }
    }

    /**
     * Sets motor voltage
     * @param voltage
     */
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    /**
     * Sets motor percent output
     * @param power
     */
    public void setPercentOutput(double power){
        motor.set(power);
    }

    public void stop(){
        motor.set(0);
    }

    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }

    public double getTicks(){
        return motor.getPosition().getValueAsDouble()*4096;
    }

    public double getCurrent(){
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public String getVelocityVoltageString(){
        return velocityVoltage.toString();
    }

    public void setFollower(TalonFXControl followerMotor, MotorAlignmentValue alignmentValue){
        motor.setControl(new Follower(followerMotor.CAN_ID, alignmentValue));
    }

    public void setForwardSoftLimit(double forwardRotations){
        motorConfig.SoftwareLimitSwitch
            .withForwardSoftLimitThreshold(forwardRotations)
            .withForwardSoftLimitEnable(true);
    }

    public void setReverseSoftLimit(double reverseRotations){
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitThreshold(reverseRotations)
            .withReverseSoftLimitEnable(true);
    }

    public void setCurrentLimit(double currentLimit){
        motorCurrentLimit.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);
        motorConfig.withCurrentLimits(motorCurrentLimit);
    }

    public void createSlot0(){
        motorPIDSlot0Configs = new Slot0Configs();
    }

    public void createSlot1(){
        motorPIDSlot1Configs = new Slot1Configs();
    }

    public void createSlot2(){
        motorPIDSlot2Configs = new Slot2Configs();
    }

    /**
     * Set motor PID configurations using many possible values
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     * @param kA
     * @param slot
     */
    public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, int slot){
        if (slot == 0){
            motorPIDSlot0Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKS(kS)
                .withKV(kV).withKA(kA);
            motorConfig.withSlot0(motorPIDSlot0Configs);
        }

        else if (slot == 1){
            motorPIDSlot1Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKS(kS)
                .withKV(kV).withKA(kA);
            motorConfig.withSlot1(motorPIDSlot1Configs);
        }

        else if (slot == 2){
            motorPIDSlot2Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKS(kS)
                .withKV(kV).withKA(kA);
            motorConfig.withSlot2(motorPIDSlot2Configs);
        }
    }

    /**
     * Set motor PID values using only P, I, D, V
     * @param kP
     * @param kI
     * @param kD
     * @param kV
     * @param slot
     */
    public void setPID(double kP, double kI, double kD, double kV, int slot){
        if (slot == 0){
            motorPIDSlot0Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKV(kV);
            motorConfig.withSlot0(motorPIDSlot0Configs);
        }

        else if (slot == 1){
            motorPIDSlot1Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKV(kV);
            motorConfig.withSlot1(motorPIDSlot1Configs);
        }

        else if (slot == 2){
            motorPIDSlot2Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKV(kV);
            motorConfig.withSlot2(motorPIDSlot2Configs);
        }
    }

    /**
     * Sets motor velocity using RPS
     * @param desiredRPM
     * @param slot
     */
    public void setVelocityControl(double desiredRPM, int slot){
        motor.setControl(velocityVoltage.withSlot(slot).withVelocity(desiredRPM / 60.0));
    }

    /**
     * Configures the motors themselves with the configuration we have done.
     */
    public void applyConfig(){
        motor.getConfigurator().apply(motorConfig);
    }

    /**
     * When set to true, forces motor to utilize brake mode behaviour during a neutral state.
     */
    public void overrideBrakeDurNeutral(boolean override){
        positionRequest.withOverrideBrakeDurNeutral(override);
    }

    /**
     * Configures whether the motor spins clockwise or counterclockwise for positive control demands
     */
    public void inverted(InvertedValue direction){
        motorConfig.MotorOutput.withInverted(direction);
    }

    /**
     * Restricts the maximum electrical current allowed to flow through a motor's stator windings
     */
    public void statorCurrentLimit(Current currentLimit){
        motorConfig.CurrentLimits.withStatorCurrentLimit(TURRET.CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    }
}

