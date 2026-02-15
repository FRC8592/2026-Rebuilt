package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.INTAKE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Intake extends SubsystemBase{
    private SparkFlex RollerMotorLeft; 
    private SparkFlex RollerMotorRight; 

    private TalonFX ExtendMotor; 

    private SparkFlexConfig rollerMotorRightConfig; 
    private SparkFlexConfig rollerMotorLeftConfig; 
    private TalonFXConfiguration extendConfiguration; 

    private SparkClosedLoopController rollerMotorRightClosedLoopController;
    private SparkClosedLoopController rollerMotorLeftClosedLoopController; 
    private PositionVoltage extendMotorController = new PositionVoltage(INTAKE.EXTEND_ROTATIONS); 

    private RelativeEncoder rollerMotorRighRelativeEncoder;
    private RelativeEncoder rollerMotorLefRelativeEncoder; 
 
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

        RollerMotorLeft = new SparkFlex (INTAKE.INTAKE_ROLLER_LEFT_CAN_ID, MotorType.kBrushless); 
        RollerMotorRight = new SparkFlex (INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, MotorType.kBrushless); 
        ExtendMotor = new TalonFX(INTAKE.INTAKE_EXTEND_CAN_ID); 

        rollerMotorLeftConfig = new SparkFlexConfig(); 
        rollerMotorRightConfig = new SparkFlexConfig(); 
        extendConfiguration = new TalonFXConfiguration(); 

        rollerMotorLeftConfig.closedLoop.pid(INTAKE.INTAKE_LEFT_P, INTAKE.INTAKE_LEFT_I, INTAKE.INTAKE_LEFT_D); 
        rollerMotorRightConfig.closedLoop.pid(INTAKE.INTAKE_RIGHT_P,INTAKE.INTAKE_RIGHT_I,INTAKE.INTAKE_RIGHT_D);

        rollerMotorLeftConfig.follow(RollerMotorRight);

        extendConfiguration.Slot0.kP = INTAKE.INTAKE_EXTEND_P; 
        extendConfiguration.Slot0.kI = INTAKE.INTAKE_EXTEND_I;
        extendConfiguration.Slot0.kD = INTAKE.INTAKE_EXTEND_D; 

        rollerMotorLeftConfig.idleMode(IdleMode.kCoast); 
        rollerMotorRightConfig.idleMode(IdleMode.kCoast);

        extendConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rollerMotorLeftConfig.smartCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT_STALL,INTAKE.INTAKE_CURRENT_LIMIT_FREE); 
        rollerMotorRightConfig.smartCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT_STALL,INTAKE.INTAKE_CURRENT_LIMIT_FREE); 

        extendConfiguration.TorqueCurrent.withPeakForwardTorqueCurrent(INTAKE.EXTEND_TORQUE_CURRENT)
        .withPeakReverseTorqueCurrent(-INTAKE.EXTEND_TORQUE_CURRENT); 

        RollerMotorLeft.configure(rollerMotorLeftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        RollerMotorRight.configure(rollerMotorRightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters); 
        ExtendMotor.getConfigurator().apply(extendConfiguration); 

        rollerMotorLeftClosedLoopController = RollerMotorLeft.getClosedLoopController(); 
        rollerMotorRightClosedLoopController = RollerMotorRight.getClosedLoopController(); 
        
        rollerMotorLefRelativeEncoder = RollerMotorLeft.getEncoder(); 
        rollerMotorRighRelativeEncoder = RollerMotorRight.getEncoder(); 
  
        // // TODO: Determine an appropriate current limit for the intake motor

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_INTAKE_LEFT", INTAKE.INTAKE_LEFT_P);
        SmartDashboard.putNumber("I_INTAKE_LEFT", INTAKE.INTAKE_LEFT_I);
        SmartDashboard.putNumber("D_INTAKE_LEFT", INTAKE.INTAKE_LEFT_D);
        SmartDashboard.putNumber("Vi_INTAKE_LEFT",INTAKE.INTAKE_LEFT_VI);

        SmartDashboard.putNumber("P_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_P);
        SmartDashboard.putNumber("I_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_I);
        SmartDashboard.putNumber("D_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_D);
        SmartDashboard.putNumber("Vi_INTAKE_RIGHT",INTAKE.INTAKE_RIGHT_VI);

        SmartDashboard.putNumber("P_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_P);
        SmartDashboard.putNumber("I_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_I);
        SmartDashboard.putNumber("D_INTAKE_EXTEND", INTAKE.INTAKE_EXTEND_D);
        SmartDashboard.putNumber("Vi_INTAKE_EXTEND",INTAKE.INTAKE_EXTEND_VI);
    }

    /**
     * Run the intake at a set speed
     */
    
    public void runAtSpeedIntake() {
        double RPMRight = SmartDashboard.getNumber("Vi_INTAKE_RIGHT", INTAKE.INTAKE_RIGHT_VI); // TODO: Remove this before competition
        rollerMotorRightClosedLoopController.setSetpoint(RPMRight, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void runToPositionExt(){
         ExtendMotor.setControl(extendMotorController.withPosition(INTAKE.DESIRED_ROTATIONS_EXTEND));
    }


    /**
     * Command to run the intake at a set speed
     */
    public Command runAtSpeedRightCommand() {
        return this.runOnce(() -> runAtSpeedIntake());
    }

    public Command runExtendCommand (){
        return this.runOnce(()->runToPositionExt()); 
    }

    /**
     * Stop the intake motor
     * 
     * We do this using voltage mode so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stop() {
        RollerMotorRight.setVoltage(0.0);
        ExtendMotor.setControl(extend_brake);
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
    
        rollerMotorRightConfig.closedLoop.pid(Right_P, Right_I, Right_D);

        RollerMotorRight.configure(rollerMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    /*
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic(){
        Logger.recordOutput("Intake Right RPM", getIntakeVelocity());
    }
        
}