package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import frc.robot.Constants.SHOOTER;

import static edu.wpi.first.units.Units.*;

import java.lang.Math;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Direction of Motors is relative to back of the shooter
    private TalonFX leftMotor;
    private TalonFX rightMotor; 
    private TalonFXConfiguration shooterMotorConfig;
    private Slot0Configs shooterPIDConfig;
    private CurrentLimitsConfigs shooterCurrentLimit;
    private FeedbackConfigs shooterFeedbackAdjustment;
    private MotionMagicConfigs shooterMMConfig;

    private VelocityVoltage shooterVV;
    private MotionMagicVelocityVoltage shooterMMVV;

    private boolean lukeisaChud;


    private double P_SET;
    private double I_SET;
    private double D_SET;



    private double targetShooterRPM;


    /**
     * Constructor for the Shooter subsystem
     * 
     * Instantiate the motor with Initial PID values from the CONSTANTS Class
     * 
     * Display PID Values on SmartDashboard
     * 
     * Set the current limit of the shooter motor
     * 
     */
    public Shooter() {

        /**
         * Flywheel and Backwheel Motor initialization and their respective configurations
         */
        leftMotor = new TalonFX(SHOOTER.LEFT_MOTOR_CAN_ID);
        rightMotor = new TalonFX(SHOOTER.RIGHT_MOTOR_CAN_ID);

        shooterMotorConfig = new TalonFXConfiguration();
        shooterPIDConfig = new Slot0Configs();
        shooterCurrentLimit = new CurrentLimitsConfigs();
        shooterFeedbackAdjustment = new FeedbackConfigs();
        shooterMMConfig = new MotionMagicConfigs();


        shooterVV = new VelocityVoltage(0);
        shooterMMVV = new MotionMagicVelocityVoltage(0);


        /**
         * Shooter PID Tuning Configuration and Constants
         */
        shooterPIDConfig
        .withKP(SHOOTER.SHOOTER_P.in(Volts))
        .withKI(SHOOTER.SHOOTER_I.in(Volts))
        .withKD(SHOOTER.SHOOTER_D.in(Volts))
        .withKS(SHOOTER.SHOOTER_S.in(Volts))
        .withKV(SHOOTER.SHOOTER_V.in(Volts))
        .withKA(SHOOTER.SHOOTER_A.in(Volts));


        shooterMotorConfig.withSlot0(shooterPIDConfig);

        // shooterMMConfig
        // .withMotionMagicAcceleration(SHOOTER.MAX_ACCELERATION)
        // .withMotionMagicJerk(SHOOTER.MAX_JERK);

        // shooterMotorConfig.withMotionMagic(shooterMMConfig);
        /**
         * Shooter Current Limit. THIS IS NOT ENABLED RIGHT NOW!
         */
        //TODO: Enable this current limit if problems!
        shooterCurrentLimit
        .withStatorCurrentLimit(SHOOTER.SHOOTER_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(false);

        shooterMotorConfig.withCurrentLimits(shooterCurrentLimit);



        //TODO: Add this back after PID Tuning if necessary
        // shooterFeedbackAdjustment.withVelocityFilterTimeConstant(SHOOTER.SHOOTER_FILTER_TIME_CONSTANT);

        // shooterMotorConfig.withFeedback(shooterFeedbackAdjustment);




        /**
         * Shooter Left Motor Configuration. This configures the motors themselves with the configuration we have done.
         */
        leftMotor.getConfigurator().apply(shooterMotorConfig);



        /**
         * Set the Shooter Right Motor to follow the Left Shooter Motor in the inverse direction
         */
        rightMotor.setControl(new Follower(SHOOTER.LEFT_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

        /**
         * SmartDashboard Flywheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("sP", SHOOTER.SHOOTER_P.in(Volts));
        SmartDashboard.putNumber("sI", SHOOTER.SHOOTER_I.in(Volts));
        SmartDashboard.putNumber("sD", SHOOTER.SHOOTER_D.in(Volts));


        SmartDashboard.putNumber("Shooter Voltage", 0);
    }



    /**
     * Run the shooter motor at a set speed in RPM.
     *  
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    // TODO: Possibly diagnose issue with inversions, IF TIME
    public void runAtSpeed(double desiredRPM) {
        double shooterMotorVelocity = desiredRPM / 60d;  // Convert from RPM to RPS for the motor
                                                        // controller

        targetShooterRPM = desiredRPM;
        //Configure the motors to run at this velocity utilizing the VelocityVoltage control modes
        leftMotor.setControl(
                shooterVV.withSlot(0).withVelocity(shooterMotorVelocity));
        //leftMotor.setControl(shooterMMVV.withVelocity(shooterMotorVelocity));
    }


    public void runAtVoltage(){
        double voltage = SmartDashboard.getNumber("Shooter Voltage", 0);
        leftMotor.setVoltage(voltage);
    }

    //This is because the Trigger is onTrue, so we are able to update continuously the shooter voltage
    public Command runAtVoltageCommand(){
        return this.run(() -> runAtVoltage());
    }

    public boolean isaChud(){
        if(Math.abs(getVelocityShooter() - targetShooterRPM) > SHOOTER.SHOOTER_TOLERANCE){
            lukeisaChud = true;
        }
        else{
            lukeisaChud = false;
        }
        return lukeisaChud;
    }


    /**
     * Stops the shooter motor, thus bringing the flywheel to a gradual stop.
     * 
     * Utilized setVoltage instead of Velocity Control to prevent power being used to stop flywheel.
     */
    public void stop() {
        leftMotor.setVoltage(0d);
    }



    /**
     * Command form of the stopShooter method.
     * 
     * @return Returns a command to run the stopShooter method once.
     */
    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }



    /**
     * Returns Flywheel Velocity
     * 
     * @return Returns velocity of the flywheel motor in RPS.
     */
    public double getVelocityShooter() {
        return leftMotor.getVelocity().getValueAsDouble();
    }


    public double getLeftMotorVoltage(){
        return leftMotor.getMotorVoltage().getValueAsDouble();
    }

    //TODO: See if the follower does provide a negative value to the Motor and change accordingly
    public double getRightMotorVoltage(){
        return -1 * rightMotor.getMotorVoltage().getValueAsDouble();
    }


    public double getLeftMotorCurrent(){
        return leftMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getRightMotorCurrent(){
        return rightMotor.getStatorCurrent().getValueAsDouble();
    }



    /**
     * Update the PID values on the fly on the shooter motor. The NEO Motors do not allowed their
     * PID Profile to be updated while running.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID() {

        //Receive Shooter PID Constants from SmartDashboard
        
        double SP_NEW = SmartDashboard.getNumber("sP", SHOOTER.SHOOTER_P.in(Volts));
        double SI_NEW = SmartDashboard.getNumber("sI", SHOOTER.SHOOTER_I.in(Volts));
        double SD_NEW = SmartDashboard.getNumber("sD", SHOOTER.SHOOTER_D.in(Volts));

        boolean FDiff = (P_SET != SP_NEW || I_SET != SI_NEW || D_SET != SD_NEW);


       if(FDiff){
        shooterPIDConfig
        .withKP(SP_NEW)
        .withKI(SI_NEW)
        .withKD(SD_NEW);
        shooterMotorConfig.withSlot0(shooterPIDConfig);
        leftMotor.getConfigurator().apply(shooterMotorConfig);

        P_SET = SP_NEW;
        I_SET = SI_NEW;
        D_SET = SD_NEW;
       }

    }



    /**
     * Periodic method, primarily for logging.
     */
    @Override
    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Set Vel", targetShooterRPM);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Real Vel", getVelocityShooter() * 60d);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Left Shooter Motor Voltage", getLeftMotorVoltage());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Right Shooter Motor Voltage", getRightMotorVoltage());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Left Flywheel Motor Current", getLeftMotorCurrent());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Right Flywheel Motor Current", getRightMotorCurrent());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Luke is a Chud(Shooter is not in Tolerance)", isaChud());
    }

}
