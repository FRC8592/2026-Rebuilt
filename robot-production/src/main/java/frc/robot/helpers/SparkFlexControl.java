package frc.robot.helpers;

import org.littletonrobotics.junction.Logger;
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
    private String motorName;

    public SparkFlexControl(int canId, boolean coastMode, String motorName){
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

        this.motorName = motorName;
    }

    public void setVelocity(double RPM){
        motorController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setPosition(double rotations){
        motorController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setVoltage(double voltage){
        motorController.setSetpoint(voltage, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    public void setPercentOutput(double power){
        motor.set(power);
    }

    public void stop(){
        motor.set(0);
    }

    public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA, double kG, double allowedError){
        motorConfig.closedLoop.p(kP).i(kI).d(kD);

        motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kG(kG);

        motorConfig.closedLoop.allowedClosedLoopError(allowedError, ClosedLoopSlot.kSlot0);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTrapezoidalMotionMagicPosition(double maxVelocity, double maxAcceleration, double allowedError){
        motorConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedProfileError(allowedError);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setInverted(){
        motorConfig.inverted(true);
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

    public double getVoltage(){
        return motor.getBusVoltage();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    public void setFollower(SparkFlexControl motorToFollow, boolean inverted){
        motorConfig.follow(motorToFollow.motor, inverted);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setForwardSoftLimit(double forwardRotations){
        motorConfig.softLimit.forwardSoftLimit(forwardRotations).forwardSoftLimitEnabled(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setReverseSoftLimit(double reverseRotations){
        motorConfig.softLimit.reverseSoftLimit(reverseRotations).reverseSoftLimitEnabled(true);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCurrentLimit(double currentLimit){
        motorConfig.secondaryCurrentLimit(currentLimit);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void motorLogging(String logPath){
        Logger.recordOutput(logPath + motorName + " RPM", getVelocity());
        Logger.recordOutput(logPath + motorName + " Rotations", getPosition());
        Logger.recordOutput(logPath + motorName + " Ticks", getTicks());
        Logger.recordOutput(logPath + motorName + " Voltage", getVoltage());
        Logger.recordOutput(logPath + motorName + " Voltage", getCurrent());
    }



    
}
