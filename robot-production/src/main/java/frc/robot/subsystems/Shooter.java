package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SHOOTER;
import javax.sound.sampled.TargetDataLine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Motor configurations
    private TalonFX flywheelMotor; // big one
    private TalonFX backwheelMotor; // smaller wheels
    private TalonFXConfiguration flywheelConfiguration;
    private TalonFXConfiguration backwheelConfiguration;
    private VelocityTorqueCurrentFOC flyWheelTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
    private VelocityTorqueCurrentFOC backWheelTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

    // Store old PID values when updating from the SmartDashboard
    private double PF_OLD;
    private double IF_OLD;
    private double DF_OLD;

    // Controls flywheel speed
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
         * 
         * The backwheel motor is now mechanically connected to the flywheel motor.
         * Backwheel PID constants are no longer needed, and the Backwheel motor will
         * be an inverted follower.
         */
        flywheelConfiguration.Slot0.kP = SHOOTER.FLYWHEEL_P;
        flywheelConfiguration.Slot0.kI = SHOOTER.FLYWHEEL_I;
        flywheelConfiguration.Slot0.kD = SHOOTER.FLYWHEEL_D;
        flywheelConfiguration.Slot0.kS = SHOOTER.FLYWHEEL_S;
        flywheelConfiguration.Slot0.kV = SHOOTER.FLYWHEEL_V;

        // Configure the flywheel motor so that positive inputs shoot fuel
        flywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;   
      
        // Configure both motors to coast instead of brake
        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        backwheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Flywheel and Backwheel Current Limits to preserve energy and help prevent brownouts
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        backwheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit = SHOOTER.FLYWHEEL_CURRENT_LIMIT;
        backwheelConfiguration.CurrentLimits.StatorCurrentLimit = SHOOTER.BACKWHEEL_CURRENT_LIMIT;

        // Set PID update frequency to maximum.  Probably unnecessary.
        flyWheelTorqueCurrentFOC.withUpdateFreqHz(1000);
        backWheelTorqueCurrentFOC.withUpdateFreqHz(1000);

        // Apply motor configurations
        flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        backwheelMotor.getConfigurator().apply(backwheelConfiguration);

        // Set backwheel as an inverted follower of the flywheel
        backwheelMotor.setControl(new Follower(SHOOTER.FLYWHEEL_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));

        /**
         * SmartDashboard Flywheel PID Constants, necessary to tune PID quickly without redeploying code
         */
        SmartDashboard.putNumber("fP", SHOOTER.FLYWHEEL_P);
        SmartDashboard.putNumber("fI", SHOOTER.FLYWHEEL_I);
        SmartDashboard.putNumber("fD", SHOOTER.FLYWHEEL_D);
    }


    /**
     * Run the shooter motor at a set speed in RPM.
     *  
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    public void runAtSpeed(double desiredRPM) {
        targetFlywheelRPM = desiredRPM / 60; // Convert from RPM to RPS for the motor controller

        //Configure the motors to run at this velocity utilizing the TorqueCurrentFOC control modes
        flywheelMotor.setControl(flyWheelTorqueCurrentFOC.withSlot(0).withVelocity(targetFlywheelRPM));
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
     * Stops motors, thus bringing the flywheel to a gradual stop.
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
     * Returns the current being drawn by the flywheel motor.
     * @return Stator current for the flywheel motor in Amperes.
     */
    public double getFlywheelCurrent(){
        return flywheelMotor.getStatorCurrent().getValueAsDouble();
    }


    /**
     * Returns the current being drawn by the backwheel motor.
     * @return Stator current for the backwheel motor in Amperes.
     */
    public double getBackwheelCurrent(){
        return backwheelMotor.getStatorCurrent().getValueAsDouble();
    }


    /**
     * Update the PID values for the shooter motor. The NEO Motors do not allow their
     * PID Profile to be updated while running, so this must only be called while disabled.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID() {
        // Read Flywheel PID Constants from SmartDashboard
        double PF = SmartDashboard.getNumber("fP", SHOOTER.FLYWHEEL_P);
        double IF = SmartDashboard.getNumber("fI", SHOOTER.FLYWHEEL_I);
        double DF = SmartDashboard.getNumber("fD", SHOOTER.FLYWHEEL_D);

        if (PF != PF_OLD || IF != IF_OLD || DF != DF_OLD) {

            flywheelConfiguration.Slot0.kP = PF;
            flywheelConfiguration.Slot0.kI = IF;
            flywheelConfiguration.Slot0.kD = DF;

            PF_OLD = PF;
            IF_OLD = IF;
            DF_OLD = DF;

            flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        }
    }


    /**
     * Periodic method, primarily for logging.
     */
    @Override
    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Set RPM", targetFlywheelRPM);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Actual RPM", getVelocityFlywheel() * 60);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Actual RPM", getVelocityBackwheel() * -1 * 60);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Motor Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Motor Voltage", backwheelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Flywheel Stator Current", getFlywheelCurrent());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Backwheel Stator Current", getBackwheelCurrent());
    }

}
