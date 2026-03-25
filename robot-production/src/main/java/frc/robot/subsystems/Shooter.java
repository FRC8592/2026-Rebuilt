package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.TURRET;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private TalonFX flywheelMotor; // big one
    private TalonFX backwheelMotor; // smaller wheels
    private TalonFXConfiguration flywheelConfiguration;
    private TalonFXConfiguration backwheelConfiguration;

    private VelocityTorqueCurrentFOC flyWheelTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
    private VelocityTorqueCurrentFOC backWheelTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);


    private double PF_OLD;
    private double IF_OLD;
    private double DF_OLD;

    private double PB_OLD;
    private double IB_OLD;
    private double DB_OLD;

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
        flywheelConfiguration = new TalonFXConfiguration();
        backwheelConfiguration = new TalonFXConfiguration();



        /**
         * Flywheel PID Tuning Configuration and Constants
         */
        flywheelConfiguration.Slot0.kP = SHOOTER.FLYWHEEL_P;
        flywheelConfiguration.Slot0.kI = SHOOTER.FLYWHEEL_I;
        flywheelConfiguration.Slot0.kD = SHOOTER.FLYWHEEL_D;
        flywheelConfiguration.Slot0.kS = SHOOTER.FLYWHEEL_S;
        flywheelConfiguration.Slot0.kV = SHOOTER.FLYWHEEL_V;



        /**
         * Backwheel PID Tuning Configuration and Constants
         */
        backwheelConfiguration.Slot0.kP = SHOOTER.BACKWHEEL_P;
        backwheelConfiguration.Slot0.kI = SHOOTER.BACKWHEEL_I;
        backwheelConfiguration.Slot0.kD = SHOOTER.BACKWHEEL_D;
        backwheelConfiguration.Slot0.kS = SHOOTER.BACKWHEEL_S;
        backwheelConfiguration.Slot0.kV = SHOOTER.BACKWHEEL_V;



        /**
         * Flywheel and Backwheel Motor Inversion Configuration
         */
        //DO NOT PAY ATTENTION TO THE INVERTEDVALUE CLASS NAME, THE ACTUAL INVERSION VALUE IS THE POSITIVE VALUE PASSED
        flywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        backwheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;



        /**
         * Flywheel and Backwheel Neutral Mode Configurations. This is to tell the motor what to do when it is at 0V.
         * (I believe so at least)
         */
        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        backwheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;



        /**
         * Flywheel and Backwheel Current Limit Configuration, this limits supply current limit too and prevents the motor from overheating
         */
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        backwheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit = SHOOTER.FLYWHEEL_CURRENT_LIMIT;
        backwheelConfiguration.CurrentLimits.StatorCurrentLimit = SHOOTER.BACKWHEEL_CURRENT_LIMIT;



        /**
         * Flywheel and Backwheel Update Speed Configuration. This is to allow the flywheels to respond quicker to errors.
         */
        // flyWheelTorqueCurrentFOC.withUpdateFreqHz(1000);
        // backWheelTorqueCurrentFOC.withUpdateFreqHz(1000);

        flywheelConfiguration.Feedback.VelocityFilterTimeConstant = 0.01;



        /**
         * Flywheel and Backwheel Motor Configuration. This configures the motors themselves with the configuration we have done.
         */
        flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        backwheelMotor.getConfigurator().apply(backwheelConfiguration);



        /**
         * SmartDashboard Flywheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("fP", SHOOTER.FLYWHEEL_P);
        SmartDashboard.putNumber("fI", SHOOTER.FLYWHEEL_I);
        SmartDashboard.putNumber("fD", SHOOTER.FLYWHEEL_D);



        /**
         * SmartDashboard Backwheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("bP", SHOOTER.BACKWHEEL_P);
        SmartDashboard.putNumber("bI", SHOOTER.BACKWHEEL_I);
        SmartDashboard.putNumber("bD", SHOOTER.BACKWHEEL_D);
    }



    /**
     * Run the shooter motor at a set speed in RPM.
     *  
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    // TODO: Possibly diagnose issue with inversions, IF TIME
    public void runAtSpeed(double desiredRPM) {
        double flyWheelMotorVelocity = desiredRPM / 60; // Convert from RPM to RPS for the motor
                                                        // controller
        double backwheelMotorVelocity = -1 * SHOOTER.BACKWHEEL_VELOCITY / 60;

        targetFlywheelRPM = desiredRPM;
        //Configure the motors to run at this velocity utilizing the TorqueCurrentFOC control modes
        flywheelMotor.setControl(
                flyWheelTorqueCurrentFOC.withSlot(0).withVelocity(flyWheelMotorVelocity));
        backwheelMotor.setControl(
                backWheelTorqueCurrentFOC.withSlot(0).withVelocity(backwheelMotorVelocity));
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
        flywheelMotor.setVoltage(0.0);
        backwheelMotor.setVoltage(0.0);
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
        double PF = SmartDashboard.getNumber("fP", SHOOTER.FLYWHEEL_P);
        double IF = SmartDashboard.getNumber("fI", SHOOTER.FLYWHEEL_I);
        double DF = SmartDashboard.getNumber("fD", SHOOTER.FLYWHEEL_D);

        //Recieve Backwheel PID Constants from SmartDashboard
        double PB = SmartDashboard.getNumber("bP", SHOOTER.BACKWHEEL_P);
        double IB = SmartDashboard.getNumber("bI", SHOOTER.BACKWHEEL_I);
        double DB = SmartDashboard.getNumber("bD", SHOOTER.BACKWHEEL_D);

        if (PF != PF_OLD || IF != IF_OLD || DF != DF_OLD
                || PB != PB_OLD || IB != IB_OLD || DB != DB_OLD) {
            flywheelConfiguration.Slot0.kP = PF;
            flywheelConfiguration.Slot0.kI = IF;
            flywheelConfiguration.Slot0.kD = DF;

            backwheelConfiguration.Slot0.kP = PB;
            backwheelConfiguration.Slot0.kI = IB;
            backwheelConfiguration.Slot0.kD = DB;

            PF_OLD = PF;
            IF_OLD = IF;
            DF_OLD = DF;

            PB_OLD = PB;
            IB_OLD = IB;
            DB_OLD = DB;


            flywheelMotor.getConfigurator().apply(flywheelConfiguration);
            backwheelMotor.getConfigurator().apply(backwheelConfiguration);
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
                SmartDashboard.getNumber("B Flywheel", SHOOTER.BACKWHEEL_VELOCITY));
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Motor Voltage",
                flywheelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Real Vel", getVelocityFlywheel() * 60);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Real Vel",
                getVelocityBackwheel() * -1 * 60);
    }

}
