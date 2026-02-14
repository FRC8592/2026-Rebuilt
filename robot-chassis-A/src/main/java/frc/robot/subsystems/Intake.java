package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.helpers.PIDProfile;


public class Intake extends SubsystemBase{

    private SparkMax RollerMotorLeft; 
    private SparkMax RollerMotorRight; 

    private TalonFX ExtendMotor; 
    
    PIDProfile RollerMotorLeftPID = new PIDProfile(); 
    PIDProfile RollerMotorRightPID = new PIDProfile(); 
    PIDProfile ExtendMotorPID = new PIDProfile(); 
    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Intake() {
        RollerMotorLeft = new SparkMax (INTAKE.INTAKE_ROLLER_LEFT_CAN_ID, MotorType.kBrushless); 
        RollerMotorRight = new SparkMax (INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, MotorType.kBrushless); 

        ExtendMotor = new TalonFX (INTAKE.INTAKE_EXTEND_CAN_ID); 


        RollerMotorLeftPID.setSlot(0);
        RollerMotorRightPID.setSlot(0);
        ExtendMotorPID.setSlot(0);

        RollerMotorLeftPID.setPID(INTAKE.INTAKE_LEFT_P, INTAKE.INTAKE_LEFT_I, INTAKE.INTAKE_LEFT_D);
        RollerMotorRightPID.setPID(INTAKE.INTAKE_RIGHT_P, INTAKE.INTAKE_RIGHT_I, INTAKE.INTAKE_RIGHT_D);
        ExtendMotorPID.setPID(INTAKE.INTAKE_EXTEND_P, INTAKE.INTAKE_EXTEND_I, INTAKE.INTAKE_EXTEND_D);
  
        // // TODO: Determine an appropriate current limit for the intake motor

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_LEFT_P);
        SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_LEFT_I);
        SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_LEFT_D);
        SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_LEFT_VI);

        SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_RIGHT_P);
        SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_RIGHT_I);
        SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_RIGHT_D);
        SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_RIGHT_VI);

        SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_EXTEND_P);
        SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_EXTEND_I);
        SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_EXTEND_D);
        SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_EXTEND_VI);
    }


    /**
     * Run the intake at a set speed
     */
    public void runAtSpeed() {
        double LeftRPM = SmartDashboard.getNumber("Vi_INTAKE_LEFT", INTAKE.INTAKE_LEFT_VI);
        
        double RightRPM = SmartDashboard.getNumber("Vi_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_VI);

        double ExtendRPM = SmartDashboard.getNumber("Vi_INTAKE_EXTEND",INTAKE.INTAKE_EXTEND_VI);
        // TODO: Remove this before competition
        // IntakeMotor.setPercentOutput(0.8); // TODO: Temporary fix to prevent oscillation.  Tune PID and run in velocity mode.
        // TODO: Tune PID for the intake to prevent oscillation.  
        // IntakeMotor.setVelocity(RPM);
    }


    /**
     * Command to run the intake at a set speed
     */
    public Command runAtSpeedCommand() {
        return this.runOnce(() -> runAtSpeed());
    }


    /**
     * Update the PID constants for the intake motor from SmartDashboard values
     * 
     * The Neo Vortex motors will not accept a change to the PID parameters while running.
     * Thusly, this method must be called from disabledPeriod() in Robot.java.
     */
    public void updatePID() {
        double Left_P = SmartDashboard.getNumber("P_INTAKE_LEFT", INTAKE.INTAKE_LEFT_P);
        double Left_I = SmartDashboard.getNumber("I_INTAKE_LEFT", INTAKE.INTAKE_LEFT_I);
        double Left_D = SmartDashboard.getNumber("D_INTAKE_LEFT", INTAKE.INTAKE_LEFT_D);
        double Left_Vi = SmartDashboard.getNumber("Vi_INTAKE_LEFT",INTAKE.INTAKE_LEFT_VI);

        double Right_P = SmartDashboard.getNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        double Right_I = SmartDashboard.getNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        double Right_D = SmartDashboard.getNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);
        double Right_Vi = SmartDashboard.getNumber("Vi_INTAKE_RIGHT",INTAKE.INTAKE_RIGHT_VI);

        double Extend_P = SmartDashboard.getNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        double Extend_I = SmartDashboard.getNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        double Extend_D = SmartDashboard.getNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);
        double Extend_Vi = SmartDashboard.getNumber("Vi_INTAKE_EXTEND",INTAKE.INTAKE_EXTEND_VI);
    
        RollerMotorLeft.setSlot(0);
        RollerMotorRight.setSlot(0); 
        ExtendMotor.setSlot(0); 

        RollerMotorLeftPID.setPID(Left_P,Left_I ,Left_D);
        RollerRightPID.setPID(Right_P,Right_I,Right_D); 
        ExtendPID.setPID(Extend_P,Extend_I,Extend_D); 
        
        }
    

    /**
     * Stop the intake motor
     * 
     * We do this using % ouptput so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stop() {
        RollerMotorLeft.setVoltage(0);
        RollerMotorRight.setVoltage(0);
        ExtendMotor.setVoltage(0);
    }


    /**
     * Stop command for the intake motor
     * @return stop command
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }


//    /**
//     * Get the velocity of the intake motor in RPM
//     * @return velocity in RPM
//     */
    public double getLeftVelocity(){
        return RollerMotorLeft.getVelocityRPM();
    }

    public double getRightVelocity(){
        return RollerMotorRight.getVelocityRPM();
    }

    public double getExtendVelocity(){
        return ExtendMotor.getVelocityRPM();
    }


    /*
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic(){
        Logger.recordOutput("RollerLeft RPM", getLeftVelocity());
        Logger.recordOutput("RollerRight RPM", getRightVelocity());
        Logger.recordOutput("RollerExtend RPM", getExtendVelocity());
     }
        
}