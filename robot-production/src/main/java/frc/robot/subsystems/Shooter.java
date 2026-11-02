package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SCORING;
import frc.robot.Constants.SHOOTER;
import frc.robot.helpers.TalonFXControl;

import java.lang.Math;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    // Direction of Motors is relative to back of the shooter
    private TalonFXControl leftMotor;
    private TalonFXControl rightMotor;

    private double P_SET;
    private double I_SET;
    private double D_SET;
    private double V_SET;

    private double targetShooterRPM;
    private double shooterSpeedOffset = 0;


    /**
     * Constructor for the Shooter subsystem
     * Instantiate the motor with Initial PID values from the CONSTANTS Class
     * Display PID Values on SmartDashboard
     * Set the current limit of the shooter motor
     */
    public Shooter() {
        leftMotor = new TalonFXControl(SHOOTER.LEFT_MOTOR_CAN_ID, true);
        rightMotor = new TalonFXControl(SHOOTER.RIGHT_MOTOR_CAN_ID, true);

        leftMotor.createSlot0();
        leftMotor.createSlot1();
        leftMotor.createSlot2();

        leftMotor.setPID(SHOOTER.SHOOTER_P, SHOOTER.SHOOTER_I, SHOOTER.SHOOTER_D, SHOOTER.SHOOTER_S, SHOOTER.SHOOTER_V_SHORT, SHOOTER.SHOOTER_A, 0);
        leftMotor.setPID(SHOOTER.SHOOTER_P, SHOOTER.SHOOTER_I, SHOOTER.SHOOTER_D, SHOOTER.SHOOTER_S, SHOOTER.SHOOTER_V_MEDIUM, SHOOTER.SHOOTER_A, 1);
        leftMotor.setPID(SHOOTER.SHOOTER_P, SHOOTER.SHOOTER_I, SHOOTER.SHOOTER_D, SHOOTER.SHOOTER_S, SHOOTER.SHOOTER_V_LONG, SHOOTER.SHOOTER_A, 2);

        leftMotor.setCurrentLimit(SHOOTER.SHOOTER_CURRENT_LIMIT);
        rightMotor.setCurrentLimit(SHOOTER.SHOOTER_CURRENT_LIMIT);

        //Set the Shooter Right Motor to follow the Left Shooter Motor in the inverse direction
        rightMotor.setFollower(leftMotor, MotorAlignmentValue.Opposed);

        SmartDashboard.putNumber("sP", SHOOTER.SHOOTER_P);
        SmartDashboard.putNumber("sI", SHOOTER.SHOOTER_I);
        SmartDashboard.putNumber("sD", SHOOTER.SHOOTER_D);
        SmartDashboard.putNumber("sV", SHOOTER.SHOOTER_V_MEDIUM);
        SmartDashboard.putNumber("Shooter Voltage", 0);
        SmartDashboard.putNumber("shooterSpeedOffset", shooterSpeedOffset);

    }

    public void runAtSpeed(double desiredRPM, int slot) {
        double shooterMotorVelocity = desiredRPM / 60d; // Convert from RPM to RPS for the motor controller
        targetShooterRPM = desiredRPM;
        Logger.recordOutput("shooterMotorRPS", shooterMotorVelocity);
        // Configure the motors to run at this velocity utilizing the VelocityVoltage control modes
        leftMotor.setVelocityControl(desiredRPM, slot);
        Logger.recordOutput("Shooter Motor Velocity Voltage Info", leftMotor.getVelocityVoltageString());
    }

    public boolean isWithin() {
        double toleranceMeasure = Math.abs(targetShooterRPM - getLeftVelocityShooter());
        if (toleranceMeasure < SHOOTER.SHOOTER_TOLERANCE)
            return true;
        else
            return false;
    }

    /**
     * Uses exponential regression to calculate speed to shoot at
     * 
     * @param distance direct distance from target
     * @return speed to shoot at 
     */
    public double shootSpeed(double distance) {
        if (distance <= 2.0){
            return 156d * distance + 1019d;
        }  else {
            return Math.pow((156d * distance + 1019d), SCORING.RANGE_EXPO);
        }
    } 
    
    public double getShooterSpeedOffset(){
        return shooterSpeedOffset;
    }

    public void increaseRPM() {
        shooterSpeedOffset += SCORING.RPM; // Orginally 10 
        SmartDashboard.putNumber("shooterSpeedOffset", shooterSpeedOffset);
    }

    /**
     * @return Command to increase RPM offset by 50
     */
    public Command increaseRPMCommand() {
        return this.runOnce(() -> increaseRPM());
    }

    public void decreaseRPM() {
        shooterSpeedOffset -= SCORING.RPM; //Orginally 10
        SmartDashboard.putNumber("shooterSpeedOffset", shooterSpeedOffset);
    }

    /**
     * @return Command to decrease RPM offset by 50
     */
    public Command decreaseRPMCommand() {
        return this.runOnce(() -> decreaseRPM());
    }

    public void stop() {
        leftMotor.setVoltage(0d);
    }

    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }

    public double getLeftVelocityShooter() {
        return leftMotor.getVelocity();
    }

    public double getRightVelocityShooter() {
        return rightMotor.getVelocity();
    }

    public double getLeftMotorVoltage() {
        return leftMotor.getVoltage();
    }

    public double getRightMotorVoltage() {
        return -1 * rightMotor.getVoltage();
    }

    public double getLeftMotorCurrent() {
        return leftMotor.getCurrent();
    }

    public double getRightMotorCurrent() {
        return rightMotor.getCurrent();
    }

    /**
     * Update the PID values for the shooter motor. The NEO Motors do not allow their PID Profile to
     * be updated while running, so this must only be called while disabled.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID() {
        double SP_NEW = SmartDashboard.getNumber("sP", SHOOTER.SHOOTER_P);
        double SI_NEW = SmartDashboard.getNumber("sI", SHOOTER.SHOOTER_I);
        double SD_NEW = SmartDashboard.getNumber("sD", SHOOTER.SHOOTER_D);
        double SV_NEW = SmartDashboard.getNumber("sV", SHOOTER.SHOOTER_V_MEDIUM);

        boolean FDiff = (P_SET != SP_NEW || I_SET != SI_NEW || D_SET != SD_NEW || V_SET != SV_NEW);

        if (FDiff) {
            leftMotor.setPID(SP_NEW, SI_NEW, SD_NEW, SV_NEW, 1);
            
            leftMotor.applyConfig();

            P_SET = SP_NEW;
            I_SET = SI_NEW;
            D_SET = SD_NEW;
            V_SET = SV_NEW;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Set Vel", targetShooterRPM);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Left Real Vel RPM",
                leftMotor.getVelocity() * 60d);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Right Real Vel RPM",
                rightMotor.getVelocity() * 60d);
        Logger.recordOutput(SHOOTER.LOG_PATH + "Left Shooter Motor Voltage", 
                leftMotor.getVoltage());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Right Shooter Motor Voltage",
                rightMotor.getVoltage());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Left Flywheel Motor Current",
                leftMotor.getCurrent());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Right Flywheel Motor Current",
                rightMotor.getCurrent());
        Logger.recordOutput(SHOOTER.LOG_PATH + "Shooter Tolerance", isWithin());
    }

}
