package frc.robot.helpers.motor.spark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
//import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor;

public abstract class SparkBaseMotor<M extends SparkBase, C extends SparkBaseConfig> extends NewtonMotor {
    protected M motor;
    protected SparkClosedLoopController motorCtrl;
    protected RelativeEncoder encoder;
    protected C config;

    private ClosedLoopSlot slot;
    private FeedForwardConfig feedForwardConfig;
    private MAXMotionConfig motionConfig;

    protected SparkBaseMotor(M motor, C config, boolean inverted){ //MotorConstants constants) {
        super(motor.getDeviceId(), inverted);
        this.motor = motor;
        this.motorCtrl = motor.getClosedLoopController();
        this.encoder = motor.getEncoder();
        this.config = config;
        this.config.inverted(inverted);
        this.feedForwardConfig = new FeedForwardConfig();
    }

    @Override
    public void setInverted(boolean inverted) {
        this.config.inverted(inverted);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        //Alternative implementation
        switch (gains.pidSlot){
            case 0:
                slot = ClosedLoopSlot.kSlot0;
            case 1:
                slot = ClosedLoopSlot.kSlot1;
            case 2:
                slot = ClosedLoopSlot.kSlot2;
            case 3:
                slot = ClosedLoopSlot.kSlot3;
        }
        // if (gains.getSlot() == 1) {
        //     slot = ClosedLoopSlot.kSlot1;
        // } else if (gains.getSlot() == 2) {
        //     slot = ClosedLoopSlot.kSlot2;
        // } else if (gains.getSlot() == 3) {
        //     slot = ClosedLoopSlot.kSlot3;
        // } else {
        //     slot = ClosedLoopSlot.kSlot0;
        // }

        feedForwardConfig.kA(gains.kA)
        .kV(gains.kV)
        .kS(gains.kS)
        .kG(gains.kG);

        this.config.closedLoop
            .p(gains.kP, slot)
            .i(gains.kI, slot)
            .d(gains.kD, slot)
            .feedForward.apply(feedForwardConfig)
            //Being deprecated so we cannot include
            //.velocityFF(gains.kV, slot)
            ;

        if (gains.softLimit) {
            this.config.softLimit.forwardSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.forwardSoftLimit(gains.softLimitMax);
            this.config.softLimit.reverseSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.reverseSoftLimit(gains.softLimitMin);
        }


        this.motionConfig = new MAXMotionConfig();
        motionConfig.cruiseVelocity(gains.maxVelocity, slot)
            .maxAcceleration(gains.maxAcceleration, slot)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot)
            .allowedProfileError(gains.tolerance, slot);

        this.config.closedLoop.apply(motionConfig);

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredVelocityRPM, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }

        slot = ClosedLoopSlot.fromInt(pidSlot);

        // double arbFF = 0d;
        // if (feedForward.size() > 0) {
        //     arbFF = feedForward.get(pidSlot).calculate(getVelocityRPM(), Robot.CLOCK.dt());
        // }
        if(motionConfig != null){        
            this.motorCtrl.setSetpoint(
            desiredVelocityRPM, 
            ControlType.kMAXMotionVelocityControl, 
            slot
            );
        }
        else{
            this.motorCtrl.setSetpoint(
                desiredVelocityRPM, 
                ControlType.kVelocity, 
                slot
            );
        }
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredRotations, 
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }

        slot = ClosedLoopSlot.fromInt(pidSlot);
        
        if(motionConfig != null){        
            this.motorCtrl.setSetpoint(
            desiredVelocityRPM, 
            ControlType.kMAXMotionPositionControl, 
            slot
            );
        }
        else{
            this.motorCtrl.setSetpoint(
                desiredVelocityRPM, 
                ControlType.kPosition, 
                slot
            );
        }
    }

    @Override
    //TODO: See if this method works and try implementing SparkMax too
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
        this.config.follow(master.getAsSparkFlex().motor);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void setCurrentLimit(int currentAmps) {
        this.config.smartCurrentLimit(currentAmps);
        this.config.secondaryCurrentLimit(currentAmps);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        com.revrobotics.spark.config.SparkBaseConfig.IdleMode mode;
        switch (idleMode) {
            case kCoast:
                mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
                break;
            case kBrake:
            // On default set to brake mode
            default:
            mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
                break;
        }

        this.config.idleMode(mode);
        this.motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public double getAppliedVoltage() {
        return motor.getBusVoltage();
    }

    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }

}