package frc.robot.helpers;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkFlexControl {
    private SparkFlex motor;
    private SparkFlexConfig motorConfig;
    private SparkClosedLoopController motorController;
    private RelativeEncoder motorEncoder;

    /**
     * @param canId motor Can ID
     * @param coastMode motor coast mode
     * Constructor for SparkFlex motors
     * Instantiates motor config, controller, and encoder
     */
    public SparkFlexControl(int canId, boolean coastMode){
        motor = new SparkFlex(canId, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorController = motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();

        if (coastMode){
            motorConfig.idleMode(IdleMode.kCoast);
        }
        else{
            motorConfig.idleMode(IdleMode.kBrake);
        }

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.set(0);

    }

    /**
     * @param RPM
     * sets motor velocity
     */
    public void setVelocity(double RPM){
        motorController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * sets motor position using rotations
     * @param rotations
     */
    public void setPosition(double rotations){
        motorController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * sets motor voltage
     * @param voltage
     */
    public void setVoltage(double voltage){
        motorController.setSetpoint(voltage, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    /**
     * sets motor percent output
     * @param power
     */
    public void setPercentOutput(double power){
        motor.set(power);
    }

    /**
     * makes motor stop by setting speed to 0
     */
    public void stop(){
        motor.set(0);
    }
    
    /**
     * sets motor PID values using all possible values
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     * @param kA
     * @param kG
     * @param allowedError
     */
    public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double allowedError){
        motorConfig.closedLoop.p(kP).i(kI).d(kD);

        motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kG(kG);

        motorConfig.closedLoop.allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot0);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets motor pid values using only P,I,D
     * @param kP
     * @param kI
     * @param kD
     */
    public void setPIDF(double kP, double kI, double kD){
        motorConfig.closedLoop.p(kP).i(kI).d(kD);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets trapezoidal motion magic
     * @param maxVelocity
     * @param maxAcceleration
     * @param allowedError
     */
    public void setTrapezoidalMotionMagicPosition(double maxVelocity, double maxAcceleration, double allowedError){
        motorConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedProfileError(allowedError);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets smartMotion
     * @param maxVelocity
     * @param maxAcceleration
     * @param allowedError
     */
     public void smartMotion(double maxVelocity, double maxAcceleration, double allowedError){
        motorConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity);
        motorConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        motorConfig.closedLoop.maxMotion.allowedProfileError(allowedError);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * makes motor inverted
     */
    public void setInverted(){
        motorConfig.inverted(true);
    }

    /**
     * @return motor velocity
     */
    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    /**
     * @return motor position
     */
    public double getPosition(){
        return motorEncoder.getPosition();
    }

    /**
     * @return gets position in ticks
     */
    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }

    /**
     * @return voltage
     */
    public double getVoltage(){
        return motor.getBusVoltage();
    }

    /**
     * @return gets current output
     */
    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    /**
     * sets follow motor
     * @param motorToFollow
     * @param inverted
     */
    public void setFollower(SparkFlexControl motorToFollow, boolean inverted){
        motorConfig.follow(motorToFollow.motor, inverted);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets the forward soft limit
     * @param forwardRotations
     */
    public void setForwardSoftLimit(double forwardRotations){
        motorConfig.softLimit.forwardSoftLimit(forwardRotations).forwardSoftLimitEnabled(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     *  sets reverse soft limit
     * @param reverseRotations
     */
    public void setReverseSoftLimit(double reverseRotations){
        motorConfig.softLimit.reverseSoftLimit(reverseRotations).reverseSoftLimitEnabled(true);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * sets current limit
     * @param currentLimit
     */
    public void setCurrentLimit(double currentLimit){
        motorConfig.secondaryCurrentLimit(currentLimit);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    
}
