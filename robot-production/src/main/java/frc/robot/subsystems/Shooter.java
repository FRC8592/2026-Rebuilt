package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER;

import static edu.wpi.first.units.Units.*;

import java.lang.Math;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final TalonFX flywheelMotor; // big one
    private final TalonFX backwheelMotor; // smaller wheels
    private final TalonFXConfiguration flywheelMotorConfig;
    private final TalonFXConfiguration backwheelMotorConfig;
    
    private final Slot0Configs flyWheelPIDConfig;
    private final Slot0Configs backWheelPIDConfig;


    private final VelocityVoltage flyWheelVV;
    private final VelocityVoltage backWheelVV;


    private double PF_SET;
    private double IF_SET;
    private double DF_SET;

    private double PB_SET;
    private double IB_SET;
    private double DB_SET;


    private double targetFlywheelRPM;


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
        flywheelMotor = new TalonFX(SHOOTER.FLYWHEEL_MOTOR_CAN_ID);
        backwheelMotor = new TalonFX(SHOOTER.BACKWHEEL_MOTOR_CAN_ID);
        flywheelMotorConfig = new TalonFXConfiguration();
        backwheelMotorConfig = new TalonFXConfiguration();


        flyWheelVV = new VelocityVoltage(0);
        backWheelVV = new VelocityVoltage(0);

        flyWheelPIDConfig = new Slot0Configs();
        backWheelPIDConfig = new Slot0Configs();

        /**
         * Flywheel PID Tuning Configuration and Constants
         */
        flyWheelPIDConfig
        .withKP(SHOOTER.FLYWHEEL_P.in(Volts))
        .withKI(SHOOTER.FLYWHEEL_I.in(Volts))
        .withKD(SHOOTER.FLYWHEEL_D.in(Volts))
        .withKS(SHOOTER.FLYWHEEL_S.in(Volts))
        .withKV(SHOOTER.FLYWHEEL_V.in(Volts))
        .withKA(SHOOTER.FLYWHEEL_A.in(Volts));



        /**
         * Backwheel PID Tuning Configuration and Constants
         */
        backWheelPIDConfig
        .withKP(SHOOTER.BACKWHEEL_P.in(Volts))
        .withKI(SHOOTER.BACKWHEEL_I.in(Volts))
        .withKD(SHOOTER.BACKWHEEL_D.in(Volts))
        .withKS(SHOOTER.BACKWHEEL_S.in(Volts))
        .withKV(SHOOTER.BACKWHEEL_V.in(Volts))
        .withKA(SHOOTER.BACKWHEEL_A.in(Volts));

        /**
         * Flywheel and Backwheel Current Limit Configuration, this limits supply current limit too and prevents the motor from overheating
         */
        flywheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        backwheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelMotorConfig.CurrentLimits.StatorCurrentLimit = SHOOTER.FLYWHEEL_CURRENT_LIMIT.in(Amps);
        backwheelMotorConfig.CurrentLimits.StatorCurrentLimit = SHOOTER.BACKWHEEL_CURRENT_LIMIT.in(Amps);




        flywheelMotorConfig.Feedback.VelocityFilterTimeConstant = 0.01;


        flywheelMotorConfig.withSlot0(flyWheelPIDConfig);
        backwheelMotorConfig.withSlot0(backWheelPIDConfig);

        /**
         * Flywheel and Backwheel Motor Configuration. This configures the motors themselves with the configuration we have done.
         */
        flywheelMotor.getConfigurator().apply(flywheelMotorConfig);
        backwheelMotor.getConfigurator().apply(backwheelMotorConfig);



        /**
         * SmartDashboard Flywheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("fP", SHOOTER.FLYWHEEL_P.in(Volts));
        SmartDashboard.putNumber("fI", SHOOTER.FLYWHEEL_I.in(Volts));
        SmartDashboard.putNumber("fD", SHOOTER.FLYWHEEL_D.in(Volts));



        /**
         * SmartDashboard Backwheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("bP", SHOOTER.BACKWHEEL_P.in(Volts));
        SmartDashboard.putNumber("bI", SHOOTER.BACKWHEEL_I.in(Volts));
        SmartDashboard.putNumber("bD", SHOOTER.BACKWHEEL_D.in(Volts));
    }



    /**
     * Run the shooter motor at a set speed in RPM.
     *  
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    // TODO: Possibly diagnose issue with inversions, IF TIME
    public void runAtSpeed(double desiredRPM) {
        double flyWheelMotorVelocity = desiredRPM / 60d;  // Convert from RPM to RPS for the motor
                                                        // controller
        double backwheelMotorVelocity = SHOOTER.BACKWHEEL_RPS_DESIRED.in(RevolutionsPerSecond);

        targetFlywheelRPM = desiredRPM;
        //Configure the motors to run at this velocity utilizing the VelocityVoltage control modes
        flywheelMotor.setControl(
                flyWheelVV.withSlot(0).withVelocity(flyWheelMotorVelocity));
        backwheelMotor.setControl(
                backWheelVV.withSlot(0).withVelocity(backwheelMotorVelocity));
    }



    /**
     * Command to run the shooter motor at a set speed.
     * 
     * @return Returns a command for running the RunAtSpeed method once.
     */
    public Command runAtSpeedCommand(double desiredRPM) {
        return this.runOnce(() -> runAtSpeed(desiredRPM));
    }



    /**
     * Stops the shooter motor, thus bringing the flywheel to a gradual stop.
     * 
     * Utilized setVoltage instead of Velocity Control to prevent power being used to stop flywheel.
     */
    public void stop() {
        flywheelMotor.setVoltage(0d);
        backwheelMotor.setVoltage(0d);
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
    public double getVelocityFlywheel() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }



    /**
     * Returns Backwheel Velocity
     * 
     * @return Returns velocity of the backwheel motor in RPS.
     */
    public double getVelocityBackwheel() {
        return backwheelMotor.getVelocity().getValueAsDouble();
    }



    /**
     * Update the PID values on the fly on the shooter motor. The NEO Motors do not allowed their
     * PID Profile to be updated while running.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID() {

        //Receive Flywheel PID Constants from SmartDashboard
        
        double PF_NEW = SmartDashboard.getNumber("fP", SHOOTER.FLYWHEEL_P.in(Volts));
        double IF_NEW = SmartDashboard.getNumber("fI", SHOOTER.FLYWHEEL_I.in(Volts));
        double DF_NEW = SmartDashboard.getNumber("fD", SHOOTER.FLYWHEEL_D.in(Volts));

        //Recieve Backwheel PID Constants from SmartDashboard
        double PB_NEW = SmartDashboard.getNumber("bP", SHOOTER.BACKWHEEL_P.in(Volts));
        double IB_NEW = SmartDashboard.getNumber("bI", SHOOTER.BACKWHEEL_I.in(Volts));
        double DB_NEW = SmartDashboard.getNumber("bD", SHOOTER.BACKWHEEL_D.in(Volts));

        boolean FDiff = (PF_SET != PF_NEW || IF_SET != IF_NEW || DF_SET != DF_NEW);
        boolean BDiff = (PB_SET != PB_NEW || IB_SET != IB_NEW || DB_SET != DB_NEW);


       if(FDiff){
        flyWheelPIDConfig
        .withKP(PF_NEW)
        .withKI(IF_NEW)
        .withKD(DF_NEW);
        flywheelMotorConfig.withSlot0(flyWheelPIDConfig);
        flywheelMotor.getConfigurator().apply(flywheelMotorConfig);
       }
      if(BDiff){
        backWheelPIDConfig
        .withKP(PB_NEW)
        .withKI(IB_NEW)
        .withKD(DB_NEW);
        backwheelMotorConfig.withSlot0(backWheelPIDConfig);
        backwheelMotor.getConfigurator().apply(backwheelMotorConfig);
      }

    }



    /**
     * Periodic method, primarily for logging.
     */
    @Override
    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Set Vel",
                targetFlywheelRPM);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Set Vel",
                SHOOTER.BACKWHEEL_RPS_DESIRED.in(RevolutionsPerSecond));
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Motor Voltage",
                flywheelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Real Vel", getVelocityFlywheel() * 60);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Real Vel",
                getVelocityBackwheel() * 60);
    }

}
