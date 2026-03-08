package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;


public class Intake extends SubsystemBase{
    private TalonFX rollerMotor;
    private SparkFlex extendMotor;

    private TalonFXConfiguration rollerConfig;
    private SparkFlexConfig extendConfig;

    private VelocityVoltage rollerMotorCtrl = new VelocityVoltage(0);
    private SparkClosedLoopController extendClosedLoopCtrl;

    private RelativeEncoder extendMotorEncoder;

    private double PR_OLD;
    private double IR_OLD;
    private double DR_OLD;

    private double PE_OLD;
    private double IE_OLD;
    private double DE_OLD;
 
    private final NeutralOut extend_brake = new NeutralOut(); 
    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Intake() {

        /*
         * Create the Intake motor and instatiate the following features
         *   Reset to safe factory configuration
         *   Place in COAST mode (Can coast to a stop)
         *   Set current limits
         *   Set VELOCITY PID parameters
         */
        rollerMotor = new TalonFX (INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID);
        rollerConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast); 
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = INTAKE.ROLLER_CURRENT_LIMIT;

        rollerConfig.Slot0.kP = INTAKE.INTAKE_RIGHT_P; 
        rollerConfig.Slot0.kI = INTAKE.INTAKE_RIGHT_I;
        rollerConfig.Slot0.kD = INTAKE.INTAKE_RIGHT_D;

        rollerMotor.getConfigurator().apply(rollerConfig); 

        /*
         * Create the Extension motor and instatiate the following features
         *   Reset to safe factory configuration
         *   Store persistant configuration (Flash)
         *   Place in Brake mode (Hold position)
         *   Set current limits
         *   Set VELOCITY PID parameters
         */
        extendMotor = new SparkFlex(INTAKE.INTAKE_EXTEND_CAN_ID, MotorType.kBrushless);         
        extendConfig = new SparkFlexConfig();

        extendConfig.idleMode(IdleMode.kBrake);
        extendConfig.smartCurrentLimit(INTAKE.EXTEND_CURRENT_LIMIT); 

        extendConfig.closedLoop.pid(INTAKE.INTAKE_EXTEND_P,INTAKE.INTAKE_EXTEND_I,INTAKE.INTAKE_EXTEND_D);

        extendMotor.configure(extendConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters); 
        extendClosedLoopCtrl = extendMotor.getClosedLoopController(); 
        
        extendMotorEncoder = extendMotor.getEncoder(); 

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        // SmartDashboard.putNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        // SmartDashboard.putNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        // SmartDashboard.putNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);
        // SmartDashboard.putNumber("Vi_INTAKE_RIGHT",INTAKE.INTAKE_RIGHT_VI);

        // SmartDashboard.putNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        // SmartDashboard.putNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        // SmartDashboard.putNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);
    }

    /**
     * Run the intake at a set speed
     */
    
    public void runToPositionExt() {
        //To run at raw power
        //rollerMotorRightClosedLoopController.setSetpoint(12, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
        //TODO: Research why Neo Motors undershoot velocity sent to the motor 
        extendClosedLoopCtrl.setSetpoint(INTAKE.EXTEND_ROTATIONS, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void runAtSpeedIntake(){
        double RPMRight = SmartDashboard.getNumber("INTAKE_VI", INTAKE.INTAKE_RIGHT_VI);
        System.out.println("Running Roller Command");
        rollerMotor.setVoltage(12);
        //rollerMotor.setControl(rollerMotorCtrl.withVelocity(RPMRight));
    }

    public void resetExtenderPos(){
        System.out.println("Resetting Extender Command");
        extendMotorEncoder.setPosition(0);
    }

    public Command resetExtenderCommand(){
        return this.runOnce(() -> resetExtenderPos());
    }

    /**
     * Command to run the intake at a predetermined speed
     */
    public Command runAtSpeedIntakeCommand() {
        return this.runOnce(() -> runAtSpeedIntake());
    }

    public Command runExtendCommand (){
        System.out.println("Extend Command is Running");
        return this.runOnce(()->runToPositionExt()); 
    }

    public double getExtendPosition(){
        return extendMotorEncoder.getPosition();
    }

    /**
     * Stop the intake motor
     * 
     * We do this using voltage mode so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stopRoller() {
        rollerMotor.setVoltage(0.0);
    }

    public void stopExtender(){
        extendMotor.setVoltage(0);
    }


    /**
     * Stop command for the intake motor
     * @return stop command
     */
    public Command stopRollerCommand(){
        return this.runOnce(() -> stopRoller());
    }

    public Command stopExtendCommand(){
        return this.runOnce(() -> stopExtender());
    }


   /**
    * Get the velocity of the intake motor in RPM
    * @return velocity in RPM
    */
    public double getIntakeVelocity(){
        return rollerMotor.getVelocity().getValueAsDouble();
    }


    /**
     * Update the PID constants for the intake motor from SmartDashboard values
     * 
     * The Neo Vortex motors will not accept a change to the PID parameters while running.
     * Thusly, this method must be called from disabledPeriod() in Robot.java.
     */
    public void updatePID() {
        double Right_P = SmartDashboard.getNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        double Right_I = SmartDashboard.getNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        double Right_D = SmartDashboard.getNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);

    
        double Extend_P = SmartDashboard.getNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        double Extend_I = SmartDashboard.getNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        double Extend_D = SmartDashboard.getNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);

        if(Right_P != PR_OLD || Right_I != IR_OLD || Right_D != DR_OLD || Extend_P != PE_OLD || Extend_I != IE_OLD || Extend_P != DE_OLD){
            rollerConfig.Slot0.kP = Right_P; 
            rollerConfig.Slot0.kI = Right_I;
            rollerConfig.Slot0.kD = Right_D; 

            
            rollerMotor.getConfigurator().apply(rollerConfig);
            extendConfig.closedLoop.pid(Extend_P, Extend_I, Extend_D);
            extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            PR_OLD = Right_P;
            IR_OLD = Right_I;
            DR_OLD = Right_D;

            PE_OLD = Extend_P;
            IE_OLD = Extend_I;
            DE_OLD = Extend_D;
            
        }

    }


    /*
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic(){
        Logger.recordOutput("Intake Right RPM", getIntakeVelocity());
        Logger.recordOutput("Extend Motor Rotations", getExtendPosition());
    }
        
}