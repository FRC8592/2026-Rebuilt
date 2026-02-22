package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private SparkFlex RollerMotorLeft; 
    private SparkFlex RollerMotorRight; 

    private TalonFX ExtendMotor; 

    private SparkFlexConfig rollerMotorRightConfig; 
    private SparkFlexConfig rollerMotorLeftConfig; 
    private TalonFXConfiguration extendConfiguration; 

    private SparkClosedLoopController rollerMotorRightClosedLoopController;
    private PositionVoltage extendMotorController = new PositionVoltage(0);
    //private PositionVoltage extendMotorController = new PositionVoltage(INTAKE.EXTEND_ROTATIONS); 

    private RelativeEncoder rollerMotorRighRelativeEncoder;

    private double P_OLD;
    private double I_OLD;
    private double D_OLD;
 
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
         *   Store persistant configuration (Flash)
         *   Place in COAST mode (Can coast to a stop)
         *   Set current limits
         *   Set VELOCITY PID parameters
         */

        //RollerMotorLeft = new SparkFlex (INTAKE.INTAKE_ROLLER_LEFT_CAN_ID, MotorType.kBrushless); 
        RollerMotorRight = new SparkFlex (INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, MotorType.kBrushless); 
        ExtendMotor = new TalonFX(INTAKE.INTAKE_EXTEND_CAN_ID); 

        //rollerMotorLeftConfig = new SparkFlexConfig(); 
        rollerMotorRightConfig = new SparkFlexConfig(); 
        extendConfiguration = new TalonFXConfiguration(); 

        //rollerMotorLeftConfig.closedLoop.pid(INTAKE.INTAKE_LEFT_P, INTAKE.INTAKE_LEFT_I, INTAKE.INTAKE_LEFT_D); 
        rollerMotorRightConfig.closedLoop.pid(INTAKE.INTAKE_RIGHT_P,INTAKE.INTAKE_RIGHT_I,INTAKE.INTAKE_RIGHT_D);
        rollerMotorRightConfig.inverted(true);

        //rollerMotorLeftConfig.follow(RollerMotorRight);

        extendConfiguration.Slot0.kP = INTAKE.INTAKE_EXTEND_P; 
        extendConfiguration.Slot0.kI = INTAKE.INTAKE_EXTEND_I;
        extendConfiguration.Slot0.kD = INTAKE.INTAKE_EXTEND_D; 

        //rollerMotorLeftConfig.idleMode(IdleMode.kCoast); 
        rollerMotorRightConfig.idleMode(IdleMode.kCoast);

        extendConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //rollerMotorLeftConfig.smartCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT_STALL,INTAKE.INTAKE_CURRENT_LIMIT_FREE); 
        rollerMotorRightConfig.smartCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT_STALL,INTAKE.INTAKE_CURRENT_LIMIT_FREE); 

        // extendConfiguration.TorqueCurrent.withPeakForwardTorqueCurrent(INTAKE.EXTEND_TORQUE_CURRENT)
        // .withPeakReverseTorqueCurrent(-INTAKE.EXTEND_TORQUE_CURRENT); 

        //RollerMotorLeft.configure(rollerMotorLeftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        RollerMotorRight.configure(rollerMotorRightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters); 
        ExtendMotor.getConfigurator().apply(extendConfiguration); 

        rollerMotorRightClosedLoopController = RollerMotorRight.getClosedLoopController(); 
        
        rollerMotorRighRelativeEncoder = RollerMotorRight.getEncoder(); 
  
        // // TODO: Determine an appropriate current limit for the intake motor

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition

        SmartDashboard.putNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        SmartDashboard.putNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        SmartDashboard.putNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);
        SmartDashboard.putNumber("Vi_INTAKE_RIGHT",INTAKE.INTAKE_RIGHT_VI);

        // SmartDashboard.putNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        // SmartDashboard.putNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        // SmartDashboard.putNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);
    }

    /**
     * Run the intake at a set speed
     */
    
    public void runAtSpeedIntake() {
        double RPMRight = SmartDashboard.getNumber("Vi_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_VI); // TODO: Remove this before competition
        //To run at raw power
        //rollerMotorRightClosedLoopController.setSetpoint(12, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
        //TODO: Research why Neo Motors undershoot velocity sent to the motor 
        rollerMotorRightClosedLoopController.setSetpoint(RPMRight, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void runToPositionExt(){
        System.out.println("Running Extender Command");
        //ExtendMotor.setVoltage(4);
         ExtendMotor.setControl(extendMotorController.withPosition(INTAKE.DESIRED_ROTATIONS_EXTEND));
    }

    public void resetExtenderPos(){
        System.out.println("Resetting Intake Command");
        ExtendMotor.setPosition(0);
    }

    public Command resetExtenderCommand(){
        return this.runOnce(() -> resetExtenderPos());
    }


    /**
     * Command to run the intake at a set speed
     */
    public Command runAtSpeedRightCommand() {
        return this.runOnce(() -> runAtSpeedIntake());
    }

    public Command runExtendCommand (){
        System.out.println("Extend Command is Running");
        return this.runOnce(()->runToPositionExt()); 
    }

    public double getExtendPosition(){
        return ExtendMotor.getPosition().getValueAsDouble();
    }

    /**
     * Stop the intake motor
     * 
     * We do this using voltage mode so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stopRoller() {
        RollerMotorRight.setVoltage(0.0);
    }

    public void stopExtender(){
        ExtendMotor.setVoltage(0);
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
        return rollerMotorRighRelativeEncoder.getVelocity();
    }


    /**
     * Update the PID constants for the intake motor from SmartDashboard values
     * 
     * The Neo Vortex motors will not accept a change to the PID parameters while running.
     * Thusly, this method must be called from disabledPeriod() in Robot.java.
     */
    public void updatePID() {
        double Right_P = SmartDashboard.getNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        double Right_I = SmartDashboard.getNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_LEFT_I);
        double Right_D = SmartDashboard.getNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);

    
        // double Extend_P = SmartDashboard.getNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        // double Extend_I = SmartDashboard.getNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        // double Extend_D = SmartDashboard.getNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);

        if(Right_P != P_OLD || Right_I != I_OLD || Right_D != D_OLD){
            extendConfiguration.Slot0.kP = Right_P; 
            extendConfiguration.Slot0.kI = Right_I;
            extendConfiguration.Slot0.kD = Right_D; 

            P_OLD = Right_P;
            I_OLD = Right_I;
            D_OLD = Right_D;

            ExtendMotor.getConfigurator().apply(extendConfiguration); 
        }

        rollerMotorRightConfig.closedLoop.pid(Right_P, Right_I, Right_D);

        RollerMotorRight.configure(rollerMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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