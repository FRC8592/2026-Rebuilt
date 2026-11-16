package frc.robot.helpers;

import org.littletonrobotics.junction.Logger;

import com.apple.eawt.event.GestureAdapter;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private boolean enabled; 

    /**
     * @param canId
     * @param coastMode
     * Constructor for TalonFX motors
     * Instantiates motor config, encoder, current limit, velocity voltage.
     */
    public TalonFXControl(int canId, boolean coastMode){
        motor = new TalonFX(canId);
        CAN_ID = canId;
        StatusCode status = motor.getDeviceTemp().refresh().getStatus();

        if (status.isOK()) {
            setEnabled(true);
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
        }

        else {
            setEnabled(false);
        }

    }

    /**
     * Sets motor position using rotations
     * @param rotations
     */

    public void setPosition(double rotations){
        if (getEnabled()) {
            PositionVoltage positionVoltage = new PositionVoltage(rotations);
            motor.setControl(positionVoltage.withPosition(rotations));
            motor.setPosition(rotations);
        }
    }

    /**
     * Sets motor voltage
     * @param voltage
     */
    public void setVoltage(double voltage){
        if (getEnabled())
        motor.setVoltage(voltage);
    }

    public void setEnabled(boolean b) {
        enabled = b;
    }

    public boolean getEnabled() {
        return enabled;
    }
    /**
     * Sets motor percent output
     * @param power
     */
    public void setPercentOutput(double power){
        if (getEnabled())
        motor.set(power);
    }

    public void stop(){
        if (getEnabled())
        motor.set(0);
    }

    public double getVelocity(){
        if (getEnabled())
        return motor.getVelocity().getValueAsDouble();

        return 0.0;
    }

    public double getVoltage(){
        if (getEnabled())
        return motor.getMotorVoltage().getValueAsDouble();

        return 0.0;
    }

    public double getPosition(){
        if (getEnabled())
        return motor.getPosition().getValueAsDouble();
        return 0.0;
    }

    public double getTicks(){
        if (getEnabled())
        return motor.getPosition().getValueAsDouble()*4096;
        return 0.0;
    }

    public double getCurrent(){
        if (getEnabled())
        return motor.getStatorCurrent().getValueAsDouble();
        return 0.0;
    }

    public String getVelocityVoltageString(){
        if (getEnabled())
        return velocityVoltage.toString();
        return "";
    }

    public void setFollower(TalonFXControl followerMotor, MotorAlignmentValue alignmentValue){
        if (getEnabled())
        motor.setControl(new Follower(followerMotor.CAN_ID, alignmentValue));
        
    }

    public void setForwardSoftLimit(double forwardRotations){
        if (getEnabled()) {

            motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardRotations;
        }
    }

    public void setReverseSoftLimit(double reverseRotations){
        if (getEnabled()) {
            motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseRotations;
        }
    }

    public void setCurrentLimit(double currentLimit){
        if (getEnabled()) {
            motorCurrentLimit.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);
            motorConfig.withCurrentLimits(motorCurrentLimit);
            applyConfig();
        }
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
        if (slot == 0 && getEnabled()){
            motorPIDSlot0Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKS(kS)
                .withKV(kV).withKA(kA);
            motorConfig.withSlot0(motorPIDSlot0Configs);
        }

        else if (slot == 1 && getEnabled()){
            motorPIDSlot1Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKS(kS)
                .withKV(kV).withKA(kA);
            motorConfig.withSlot1(motorPIDSlot1Configs);
        }

        else if (slot == 2 && getEnabled()){
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
        if (slot == 0 && getEnabled()){
            motorPIDSlot0Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKV(kV);
            motorConfig.withSlot0(motorPIDSlot0Configs);
        }

        else if (slot == 1 && getEnabled()){
            motorPIDSlot1Configs.withKP(kP).withKI(kI)
                .withKD(kD).withKV(kV);
            motorConfig.withSlot1(motorPIDSlot1Configs);
        }

        else if (slot == 2 && getEnabled()){
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
        if (getEnabled())
        motor.setControl(velocityVoltage.withSlot(slot).withVelocity(desiredRPM / 60.0));
    }

    /**
     * Configures the motors themselves with the configuration we have done.
     */
    public void applyConfig(){
        if (getEnabled())
        motor.getConfigurator().apply(motorConfig);
    }
}

