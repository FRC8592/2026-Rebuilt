package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;
import frc.robot.helpers.motor.talonfx.TalonFXMotor;


public class Intake extends SubsystemBase{

    private SparkFlexMotor RollerMotorLeft; 
    private SparkFlexMotor RollerMotorRight; 

    private TalonFXMotor ExtendMotor; 

    private PIDProfile RollerLeftPID;
    private PIDProfile RollerRightPID; 
    private PIDProfile ExtendMotorPID; 
 
    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Intake() {

        RollerMotorLeft = new SparkFlexMotor(INTAKE.INTAKE_ROLLER_MOTOR_LEFT_CAN_ID, false); 
        RollerMotorRight = new SparkFlexMotor(INTAKE.INTAKE_ROLLER_MOTOR_RIGHT_CAN_ID, false); 

        ExtendMotor = new TalonFXMotor(INTAKE.INTAKE_EXTEND_CAN_ID, false ); 
    

        RollerMotorLeftPID = new PIDProfile(); 
        RollerMotorRightPID = new PIDProfile(); 
        ExtendMotorPID = new PIDProfile(); 

        RollerLeftPID.setSlot(0);
        RollerRightPID.setSlot(0);
        ExtendMotorPID.set(0);

        RollerLeftPID.setPID(INTAKE.INTAKE_LEFT_P, INTAKE.INTAKE_LEFT_I, INTAKE.INTAKE_LEFT_D);
        RollerRightPID.setPID(INTAKE.INTAKE_RIGHT_P, INTAKE.INTAKE_RIGHT_I, INTAKE.INTAKE_RIGHT_D);
        ExtendMotorPID.setPID(INTAKE.INTAKE_EXTEND_P, INTAKE.INTAKE_EXTEND_I, INTAKE.INTAKE_EXTEND_D);

        RollerMotorLeft.withGains(RollerLeftPID);
        RollerMotorRight.withGains(RollerRightPID); 
        ExtendMotor.withGains(ExtendMotorPID); 


        // TODO: Set idle mode
        RollerMotorRight.setIdleMode(IdleMode.kCoast);
        RollerMotorLeft.setIdleMode(IdleMode.kCoast);
        ExtendMotor.setIdleMode(IdleMode.kCoast);
  
        // TODO: Determine an appropriate current limit for the intake motor
        RollerMotorLeft.setCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT);
        RollerMotorRight.setCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT);
        ExtendMotor.setCurrentLimit(INTAKE.INTAKE_EXTEND_LIMIT);

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

        RollerLeftPID.setPID(Left_P,Left_I ,Left_D);
        RollerRightPID.setPID(Right_P,Right_I,Right_D); 
        ExtendPID.setPID(Extend_P,Extend_I,Extend_D); 

        RollerMotorLeft.withGains(RollerLeftPID);
        RollerMotorRight.withGains(RollerRightPID); 
        ExtendMotor.withGains(ExtendMotorPID); 
        }
    

    /**
     * Stop the intake motor
     * 
     * We do this using % ouptput so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stop() {
        RollerMotorLeft.setPercentOutput(0);
        RollerMotorRight.setPercentOutput(0);
        ExtendMotor.setPercentOutput(0);
    }


    /**
     * Stop command for the intake motor
     * @return stop command
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }


   /**
    * Get the velocity of the intake motor in RPM
    * @return velocity in RPM
    */
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