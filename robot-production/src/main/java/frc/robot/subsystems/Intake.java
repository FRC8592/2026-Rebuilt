package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.TalonFXControl;

public class Intake extends SubsystemBase {
    private TalonFXControl rollerRightMotor;
    private TalonFXControl rollerLeftMotor;
    private SparkFlexControl extendMotor;
    private double retractionPosition;

    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Intake() {

        /*
         * Create the Intake motor and instatiate the following features Reset to safe factory
         * configuration Place in COAST mode (Can coast to a stop) Set current limits Set VELOCITY
         * PID parameters
         */
        rollerRightMotor = new TalonFXControl(INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, true);
        rollerLeftMotor = new TalonFXControl(INTAKE.INTAKE_MOTOR_LEFT_CAN_ID, true);

        rollerLeftMotor.setFollower(rollerRightMotor, MotorAlignmentValue.Opposed);

        /*
         * Create the Extension motor and instatiate the following features Reset to safe factory
         * configuration Store persistant configuration (Flash) Place in Brake mode (Hold position)
         * Set current limits Set VELOCITY PID parameters
         */
        extendMotor = new SparkFlexControl(INTAKE.INTAKE_EXTEND_CAN_ID, true);

        extendMotor.setCurrentLimit(INTAKE.EXTEND_CURRENT_LIMIT);

        extendMotor.setPIDF(INTAKE.INTAKE_EXTEND_P, INTAKE.INTAKE_EXTEND_I, INTAKE.INTAKE_EXTEND_D);

        extendMotor.smartMotion(INTAKE.CRUISE_VELOCITY,INTAKE.MAX_ACCELERATION,10);
        extendMotor.setReverseSoftLimit(INTAKE.EXTEND_SOFT_LIMIT);
    }

    /**
     * Extend the intake at controlled speed
     */
    public void extendIntake() {
        // TODO: Research why Neo Motors undershoot velocity sent to the motor
        if (getExtendPosition() > INTAKE.EXTEND_ROTATIONS){
            extendMotor.setVoltage(6);
        }
        else{
            extendMotor.setPosition(INTAKE.EXTEND_ROTATIONS);
        }
    }

    /**
     * Retract the intake at controlled speed
     */
    public void retractIntake() {
        retractionPosition += INTAKE.RETRACT_ROTATION_INCREMENT;

        extendMotor.setVoltage(-6);
    }

    /**
     * Run the intake rollers. Currently under simple voltage control
     */
    public void runIntakeRollers() {
        System.out.println("Running Roller Command");
        rollerRightMotor.setVoltage(11);
    }

    public void runIntakeRollersSlower(){
        rollerRightMotor.setVoltage(7);
    }

    public Command runIntakeRollersSlowerCommand(){
        return this.runOnce(() -> runIntakeRollersSlower());
    }

    public void runReversedIntakeRollers() {
        rollerRightMotor.setVoltage(-11);
    }

    public void retractWithRollers() {
        runIntakeRollersSlower();
        retractIntake();
    }

    public Command retractWithRollersCommand() {
        return this.runOnce(() -> retractWithRollers());
    }

    public double getRightIntakeVoltage() {
        return rollerRightMotor.getVoltage();
    }

    public double getLeftIntakeVoltage() {
        return rollerLeftMotor.getVoltage();
    }

    /**
     * Command to extend the intake at controlled speed
     */
    public Command extendIntakeCommand() {
        return this.runOnce(() -> extendIntake());
    }

    /**
     * Command to retract the intake at controlled speed
     */
    public Command retractIntakeCommand() {
        return this.runOnce(() -> retractIntake());
    }

    /**
     * Command to run the intake rollers. Currently under simple voltage control
     */
    public Command runIntakeRollersCommand() {
        return this.runOnce(() -> runIntakeRollers());
    }

    public Command runReversedIntakeRollersCommand() {
        return this.runOnce(() -> runReversedIntakeRollers());
    }

    public double getExtendPosition() {
        return extendMotor.getPosition();
    }

    /**
     * Get the velocity of the intake motor in RPM
     * 
     * @return velocity in RPM
     */
    public double getIntakeVelocity() {
        return rollerRightMotor.getVelocity();
    }

    /**
     * Stop the intake motor
     * 
     * We do this using voltage mode so that the motor will slow to a stop naturally. 
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stopRoller() {
        rollerRightMotor.setVoltage(0d);
    }

    public void stopExtender() {
        extendMotor.setVoltage(0d);
    }

    /**
     * Stop command for the intake motor
     * 
     * @return stop command
     */
    public Command stopRollerCommand() {
        return this.runOnce(() -> stopRoller());
    }

    public Command stopExtendCommand() {
        return this.runOnce(() -> stopExtender());
    }

    /**
     * Update the PID constants for the intake motor from SmartDashboard values
     * 
     * The Neo Vortex motors will not accept a change to the PID parameters while running. Thusly,
     * this method must be called from disabledPeriod() in Robot.java.
     */

    /*
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic() {
        Logger.recordOutput(INTAKE.LOG_PATH + "Intake Right RPM", getIntakeVelocity() * 60d);
        Logger.recordOutput(INTAKE.LOG_PATH + "Extend Motor Rotations", getExtendPosition());
        Logger.recordOutput(INTAKE.LOG_PATH + "Retraction Position", retractionPosition);
        Logger.recordOutput(INTAKE.LOG_PATH + "Right Roller Motor Voltage",getRightIntakeVoltage());
        Logger.recordOutput(INTAKE.LOG_PATH + "Left Roller Motor Voltage", getLeftIntakeVoltage());
        Logger.recordOutput(INTAKE.LOG_PATH + "Extend Motor Velocity", extendMotor.getVelocity());
    }
}
