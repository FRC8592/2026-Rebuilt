package frc.robot.helpers.motor.talonfx;



import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
//import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor;

public abstract class TalonFXMotor extends NewtonMotor {
    protected TalonFX motor;

    private TalonFXConfiguration configuration;

    private PositionVoltage positionOutput;
    private VelocityVoltage velocityOutput;
    private DutyCycleOut percentOutput;
    private VoltageOut voltageOutput;
    private MotionMagicVoltage motionMagicOutput;
    private MotionMagicVelocityVoltage motionMagicVelocityOutput;
    private MotionMagicConfigs motionMagicConfig;

    public TalonFXMotor(int motorID) {
        this(motorID, false);
    }

    public TalonFXMotor(int motorID, boolean inverted) {
        super(motorID, inverted);
        this.motor = new TalonFX(motorID);
        this.configuration = new TalonFXConfiguration();
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;
        //Alternate implementation:
        //this.configuration.MotorOutput.Inverted = InvertedValue.valueOf(inverted? 1: 0);

        this.motor.getConfigurator().apply(configuration);

        this.positionOutput = new PositionVoltage(0.0);
        this.velocityOutput = new VelocityVoltage(0.0);
        this.voltageOutput = new VoltageOut(0);
        this.percentOutput = new DutyCycleOut(0);
        this.motionMagicConfig = configuration.MotionMagic;
        //TODO: Discuss whether these should be removed. They utilize power to stop the motor, but generate a lot of heat
        //Need to research the behavior of it more
        this.percentOutput.OverrideBrakeDurNeutral = true;
        this.positionOutput.OverrideBrakeDurNeutral = true;
        this.velocityOutput.OverrideBrakeDurNeutral = true;
        this.motionMagicOutput.OverrideBrakeDurNeutral = true;
    }
    /*This method is exactly like the TalonFX examples provided, it takes a motionmagic profile from TalonFX configuration
      and applies all acceleration, cruise velocity, and jerk and applies that to motor config   
    */
    //CHANGES: created a maxJerk constant so the jerk variable can be changed in any case
    public void configureMotionMagic(double maxAcceleration, double cruiseVelocity, double maxJerk){
        this.motionMagicOutput = new MotionMagicVoltage(0.0);
        this.motionMagicVelocityOutput = new MotionMagicVelocityVoltage(0.0);
        motionMagicConfig.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(maxAcceleration));
        motionMagicConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(cruiseVelocity));
        motionMagicConfig.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(maxJerk));
        motor.getConfigurator().apply(configuration);
    }
    //This method is if people do not care about the jerk, just want to use the default value because it works
    public void configureMotionMagic(double maxAcceleration, double cruiseVelocity){
        this.configureMotionMagic(maxAcceleration, cruiseVelocity, 100 * maxAcceleration);
    }

    //TODO: Implement Motion Magic Expo for flywheels on Shooter
    public void configureMotionMagicExpo(double kV, double kA){
        this.motionMagicOutput = new MotionMagicVoltage(0.0);
        this.motionMagicVelocityOutput = new MotionMagicVelocityVoltage(0.0);
        motionMagicConfig.withMotionMagicExpo_kV(Volts.per(RotationsPerSecond).ofNative(kV));
        motionMagicConfig.withMotionMagicExpo_kA(Volts.per(RotationsPerSecondPerSecond).ofNative(kA));
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        //Different more conventional type of implementation
        // if(inverted){
        //     configuration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        // }
        // else{
        //     configuration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        // }  
        
        //More streamlined alternate implementation
        //this.configuration.MotorOutput.Inverted = InvertedValue.valueOf(inverted? 1: 0);

        this.motor.getConfigurator().apply(configuration);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        //TODO: Maybe implement this so we do not have to do switch case?
        //Seems like switch case is redundant but only way to do this unless we change more things
        // try{
        //     Class slotConfig = Class.forName("Slot" + gains.getSlot() + "Configs");

        // }
        // catch(Exception e){
        //     System.out.println("Class name not found!");
        // }
        
        //Added a kG value to each case, did not change method chaining as it is the best way to do this.
        //Included alternate more conventional implementation if we do want to do that, although I believe it is not necessary
        // switch (gains.pidSlot) {
        // No need for a default case as it is automatically initialized at 0
        //     case 0:
        //         Slot0Configs slot0Config = configuration.Slot0;
        //         slot0Config.kP = gains.kP;
        //         slot0Config.kI = gains.kI;
        //         slot0Config.kD = gains.kD;
        //         slot0Config.kA = gains.kA;
        //         slot0Config.kV = gains.kV;
        //         slot0Config.kS = gains.kS;
        //         slot0Config.kG = gains.kG;

        //         this.motor.getConfigurator().apply(slot0Config);
        //     case 1:
        //         Slot1Configs slot1Config = configuration.Slot1;
        //         slot1Config.kP = gains.kP;
        //         slot1Config.kI = gains.kI;
        //         slot1Config.kD = gains.kD;
        //         slot1Config.kA = gains.kA;
        //         slot1Config.kV = gains.kV;
        //         slot1Config.kS = gains.kS;
        //         slot1Config.kG = gains.kG;

        //         this.motor.getConfigurator().apply(slot1Config);
        //         break;
        //     case 2:
        //         Slot2Configs slot2Config = configuration.Slot2;
        //         slot2Config.kP = gains.kP;
        //         slot2Config.kI = gains.kI;
        //         slot2Config.kD = gains.kD;
        //         slot2Config.kA = gains.kA;
        //         slot2Config.kV = gains.kV;
        //         slot2Config.kS = gains.kS;
        //         slot2Config.kG = gains.kG;

        //         this.motor.getConfigurator().apply(slot2Config);
        //         break;
        // }
        
        //Original implementation of this, utilizes method chaining
        switch (gains.pidSlot) {
            case 0:
                Slot0Configs slot0Config = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                this.motor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = configuration.Slot1
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                this.motor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = configuration.Slot2
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                this.motor.getConfigurator().apply(slot2Config);
                break;
            default:
                Slot0Configs slotConfig = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS)
                    .withKG(gains.kG);

                this.motor.getConfigurator().apply(slotConfig);
                break;
        }
        
    }

    @Override
    public void setPercentOutput(double percent) {
        //Alternative implementation:
        // percentOutput.Output = percent;
        // this.motor.setControl(percentOutput);
        this.motor.setControl(percentOutput.withOutput(percent));
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        //Alternative implementation:
        //voltageOutput = voltage;
        //this.motor.setControl(voltageOutput);
        this.motor.setVoltage(voltage);
        //TODO: Remove this and see if it breaks implementation
        motor.getMotorVoltage();
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
        double desiredRPS = desiredRPM / 60.0;
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRPS, 
                motorPIDGains.get(pidSlot).maxNegVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        //TODO: Look properly at the implementation of the withSlot method
        if(motionMagicVelocityOutput != null){
            this.motor.setControl(motionMagicVelocityOutput.withSlot(pidSlot).withVelocity(desiredRPS));
        }
        else{
        this.motor.setControl(velocityOutput.withSlot(pidSlot).withVelocity(desiredRPS));
        }
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRotations,
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        //TODO: Look properly at the implementation of the withSlot method
        if(motionMagicOutput != null){
            this.motor.setControl(motionMagicOutput.withSlot(pidSlot).withPosition(desiredRotations));
        }
        else{
        this.motor.setControl(positionOutput.withSlot(pidSlot).withPosition(desiredRotations));
        }
    }

    @Override
    //Fixed setFollowerTo method: Ashwin Potluri
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
        Follower followObject = new Follower(master.getAsTalonFX().getDeviceID(), MotorAlignmentValue.valueOf(reversed? 1: 0));
        this.motor.setControl(followObject);
    }

    @Override
    public void setCurrentLimit(int currentAmps) {

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = currentAmps;
        currentConfigs.StatorCurrentLimitEnable = true;

        this.configuration.CurrentLimits = currentConfigs;
        this.motor.getConfigurator().apply(configuration);
    }

    /*
     * Alternate Implementation of Current Limits:
       @Override
        public void setCurrentLimit(int currentMax) {
            setCurrentLimit(currentMax, currentMax);
        }

       @Override
        public void setCurrentLimit(int motorLimit, int supplyLimit){
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
            currentLimits.withStatorCurrentLimit(motorLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyLimit)
            .withSupplyCurrentLimitEnable(true);
            configuration.withCurrentLimits(currentLimits);
            this.motor.getConfigurator().apply(configuration);
        }
     */

    public void setPositionSoftLimit(double low, double high) {

        SoftwareLimitSwitchConfigs soft_limit_motor = new SoftwareLimitSwitchConfigs();
        // Alternate implementation
        // soft_limit_motor.withForwardSoftLimitThreshold(high)
        // .withForwardSoftLimitEnable(true)
        // .withReverseSoftLimitThreshold(low)
        // .withReverseSoftLimitEnable(true);
        soft_limit_motor.ForwardSoftLimitEnable = true;
        soft_limit_motor.ReverseSoftLimitEnable = true;

        soft_limit_motor.ForwardSoftLimitThreshold = high;
        soft_limit_motor.ReverseSoftLimitThreshold = low;

        this.configuration.withSoftwareLimitSwitch(soft_limit_motor);

        this.motor.getConfigurator().apply(this.configuration);

    }

    //Alternate implementation, changing after initialization perhaps
    public void setPositionSoftLimit(PIDProfile gains, int slot){
        if(super.motorPIDGains.get(slot).softLimit){
          SoftwareLimitSwitchConfigs soft_limit_motor = new SoftwareLimitSwitchConfigs();
          soft_limit_motor.withForwardSoftLimitThreshold(gains.softLimitMax)
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(gains.softLimitMin)
          .withReverseSoftLimitEnable(true);

          this.configuration.withSoftwareLimitSwitch(soft_limit_motor);
          this.motor.getConfigurator().apply(configuration);
        }
    }

    @Override
    //Fixed the setIdleMode so there is the option of using coast mode
    public void setIdleMode(IdleMode idleMode) {
        NeutralModeValue neutralMode = NeutralModeValue.Brake;
        switch(idleMode) {
            case kCoast:
                neutralMode = NeutralModeValue.Coast;
                break;
            case kBrake:
                neutralMode = NeutralModeValue.Brake; 
                break;
        }
        this.motor.setNeutralMode(neutralMode);
    }


    @Override
    public double getVelocityRPM() {
        return this.motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getAppliedVoltage(){
        return this.motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getRotations() {
        return this.motor.getPosition().getValueAsDouble();
    }


    @Override
    public void resetEncoderPosition(double rotations) {
        this.motor.setPosition(rotations);
    }



}