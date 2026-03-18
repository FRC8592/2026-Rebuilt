package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private TalonFX rollerRightMotor;
    private TalonFX rollerLeftMotor;
    private SparkFlex extendMotor;

    private TalonFXConfiguration rollerRightConfig;
    private TalonFXConfiguration rollerLeftConfig;
    private SparkFlexConfig extendConfig;

    private VelocityVoltage rollerMotorCtrl = new VelocityVoltage(0);
    private SparkClosedLoopController extendClosedLoopCtrl;

    private RelativeEncoder extendMotorEncoder;

    private double PR_OLD = INTAKE.INTAKE_RIGHT_P;
    private double IR_OLD = INTAKE.INTAKE_RIGHT_I;
    private double DR_OLD = INTAKE.INTAKE_RIGHT_D;

    private double PE_OLD = INTAKE.INTAKE_EXTEND_P;
    private double IE_OLD = INTAKE.INTAKE_EXTEND_I;
    private double DE_OLD = INTAKE.INTAKE_EXTEND_D;

    private double extendMotorVoltage;

    private double retractionPosition;
 
    //private final NeutralOut extend_brake = new NeutralOut();
    
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
        rollerRightMotor = new TalonFX (INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID);
        rollerLeftMotor = new TalonFX(INTAKE.INTAKE_MOTOR_LEFT_CAN_ID);
        rollerRightConfig = new TalonFXConfiguration();
        rollerLeftConfig = new TalonFXConfiguration();

        //TODO: Remove this, should not be necessary
        rollerRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerRightConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast); 
        rollerRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerRightConfig.CurrentLimits.StatorCurrentLimit = INTAKE.ROLLER_CURRENT_LIMIT;

        rollerRightConfig.Slot0.kP = INTAKE.INTAKE_RIGHT_P; 
        rollerRightConfig.Slot0.kI = INTAKE.INTAKE_RIGHT_I;
        rollerRightConfig.Slot0.kD = INTAKE.INTAKE_RIGHT_D;

        rollerRightMotor.getConfigurator().apply(rollerRightConfig);

        rollerLeftConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast); 
        rollerLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerLeftConfig.CurrentLimits.StatorCurrentLimit = INTAKE.ROLLER_CURRENT_LIMIT;

        rollerLeftMotor.getConfigurator().apply(rollerLeftConfig);

        rollerLeftMotor.setControl(new Follower(INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, MotorAlignmentValue.Opposed));

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

        extendConfig.idleMode(IdleMode.kCoast);
        extendConfig.smartCurrentLimit(INTAKE.EXTEND_CURRENT_LIMIT); 

        extendConfig.closedLoop.pid(INTAKE.INTAKE_EXTEND_P,INTAKE.INTAKE_EXTEND_I,INTAKE.INTAKE_EXTEND_D);
        extendConfig.closedLoop.maxMotion.cruiseVelocity(INTAKE.CRUISE_VELOCITY);
        extendConfig.closedLoop.maxMotion.maxAcceleration(INTAKE.MAX_ACCELERATION);
        extendConfig.closedLoop.maxMotion.allowedProfileError(10);

        extendMotor.configure(extendConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters); 
        extendClosedLoopCtrl = extendMotor.getClosedLoopController(); 
        
        extendMotorEncoder = extendMotor.getEncoder(); 

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        SmartDashboard.putNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        SmartDashboard.putNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);
        SmartDashboard.putNumber("Vi_INTAKE_RIGHT",INTAKE.INTAKE_RIGHT_VI);

        SmartDashboard.putNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        SmartDashboard.putNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        SmartDashboard.putNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);

        SmartDashboard.putNumber("Retraction Voltage", 0);

        SmartDashboard.putNumber("Intake Roller Voltage", 2);
    }


    /**
     * Extend the intake at controlled speed
     */
    public void extendIntake() {
        //To run at raw power
        //rollerMotorRightClosedLoopController.setSetpoint(12, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
        //TODO: Research why Neo Motors undershoot velocity sent to the motor 
        extendClosedLoopCtrl.setSetpoint(INTAKE.EXTEND_ROTATIONS, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        retractionPosition = getExtendPosition();
    }


    /**
     * Retract the intake at controlled speed
     */
    public void retractIntake(double voltage){
        Logger.recordOutput("Voltage to Extend Motor", voltage);
        retractionPosition -= INTAKE.RETRACT_ROTATION_INCREMENT;
        if(getExtendPosition() >= 0.25) {
            extendClosedLoopCtrl.setSetpoint(voltage, ControlType.kVoltage);
            //extendClosedLoopCtrl.setSetpoint(getExtendPosition() - retractionPosition, ControlType.kMAXMotionPositionControl);
            //extendClosedLoopCtrl.setSetpoint(retractionPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }
    }


    /**
     * Run the intake rollers.  Currently under simple voltage control
     */    
    public void runIntakeRollers(){
        //TODO: Delete this! This is only for testing purposes!
        //double IntakeVoltage = SmartDashboard.getNumber("Intake Motor Voltage", 2);
        //double RPMRight = SmartDashboard.getNumber("INTAKE_VI", INTAKE.INTAKE_RIGHT_VI);
        System.out.println("Running Roller Command");
        rollerRightMotor.setVoltage(11);
        //rollerMotor.setControl(rollerMotorCtrl.withVelocity(RPMRight));
    }

    public void runReversedIntakeRollers(){
        rollerRightMotor.setVoltage(-11);
    }


    /**
     * Reset the position of the intake extension motor to 0
     */
    public void resetExtenderPos(){
        System.out.println("Resetting Extender Command");
        extendMotorEncoder.setPosition(2);
    }

    public double getRightIntakeVoltage(){
        return rollerRightMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getLeftIntakeVoltage(){
        return rollerLeftMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Command to extend the intake at controlled speed
     */
    public Command extendIntakeCommand() {
        return this.run(() -> extendIntake());
    }

    /**
     * Command to retract the intake at controlled speed
     */
    public Command retractIntakeCommand(double voltage){
        return this.runOnce(() -> retractIntake(voltage));
    }


    /**
     * Command to run the intake rollers.  Currently under simple voltage control
     */
    public Command runIntakeRollersCommand() {
        return this.runOnce(() -> runIntakeRollers());
    }

    public Command runReversedIntakeRollersCommand(){
        return this.runOnce(() -> runReversedIntakeRollers());
    }

    /**
     * Command to reset the position of the intake extension motor to 0
     */
    public Command resetExtenderCommand(){
        return this.runOnce(() -> resetExtenderPos());
    }


    // public void setCoastMode(){
    //     extendConfig.idleMode(IdleMode.kCoast);
    //     extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // }

    // public void setBrakeMode(){
    //     extendConfig.idleMode(IdleMode.kBrake);
    //     extendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // }

    public double getExtendPosition(){
        return extendMotorEncoder.getPosition();
    }


    /**
    * Get the velocity of the intake motor in RPM
    * @return velocity in RPM
    */
    public double getIntakeVelocity(){
        return rollerRightMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Stop the intake motor
     * 
     * We do this using voltage mode so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stopRoller() {
        rollerRightMotor.setVoltage(0.0);
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

        if(Right_P != PR_OLD || Right_I != IR_OLD || Right_D != DR_OLD || Extend_P != PE_OLD || Extend_I != IE_OLD || Extend_D != DE_OLD){
            rollerRightConfig.Slot0.kP = Right_P; 
            rollerRightConfig.Slot0.kI = Right_I;
            rollerRightConfig.Slot0.kD = Right_D; 

            
            rollerRightMotor.getConfigurator().apply(rollerRightConfig);
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
        Logger.recordOutput(INTAKE.LOG_PATH + "Intake Right RPM", getIntakeVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH + "Extend Motor Rotations", getExtendPosition());
        Logger.recordOutput(INTAKE.LOG_PATH + "Retraction Position", retractionPosition);
        Logger.recordOutput(INTAKE.LOG_PATH + "Right Roller Motor Voltage", getRightIntakeVoltage());
        Logger.recordOutput(INTAKE.LOG_PATH + "Left Roller Motor Voltage", getLeftIntakeVoltage());
    }
        
}