package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;
//import frc.robot.subsystems.AutoTurretAngle;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;


public class Turret extends SubsystemBase{
    // Motor for turret rotation
    private TalonFX tMotor;
    private TalonFXConfiguration tMotorConfiguration;
    private PositionVoltage positionRequest;
    private PositionTorqueCurrentFOC positionTorqueCurrent;
    //private MotionMagicVoltage motionMagicRequest;
    // Absolute encoders used to find the starting position of the turret
    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;
    private double E1_value;
    private double E2_value;
    // Calculate turret angle based on a target location and the robot's current position
    private AutoTurretAngle angleCalc;
    private static Map<Double, Double> map = new HashMap<Double, Double>();
    private static Set<Double> set = new HashSet<Double>();


    private double P_OLD;
    private double I_OLD;
    private double D_OLD;
    private double S_OLD;

    public Turret() {
        // Instantiate the absolute encoders and get our starting position
        E1 = new DutyCycleEncoder(0, 360, 0);
        E2 = new DutyCycleEncoder(1, 360, 0);
        E1_value = E1.get();
        E2_value = E2.get();

        // Create the turret motor, configuration object and controller
        tMotor = new TalonFX(TURRET.TURRET_MOTOR_CAN_ID);
        tMotorConfiguration = new TalonFXConfiguration();
        positionRequest = new PositionVoltage(0);

        // Put motor in brake mode, invert, and apply current limits
        tMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        tMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        tMotorConfiguration.CurrentLimits.StatorCurrentLimit = TURRET.TURRET_CURRENT_LIMIT;
        tMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set soft limits
        tMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET.FORWARD_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS;
        tMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET.REVERSE_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS;
        // Apply soft limits to help avoid driving the turret past the cable extension
        tMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        tMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Apply soft limits to help avoid driving the turret past the cable extension

        // Configure PID controls and Motion Magic parameters
        tMotorConfiguration.Slot0.kP = TURRET.TURRET_P0; 
        tMotorConfiguration.Slot0.kI = TURRET.TURRET_I0;
        tMotorConfiguration.Slot0.kD = TURRET.TURRET_D0; 
        tMotorConfiguration.Slot0.kS = TURRET.TURRET_S;
        tMotorConfiguration.Slot1.kP = TURRET.TURRET_P1; 
        tMotorConfiguration.Slot1.kI = TURRET.TURRET_I1;
        tMotorConfiguration.Slot1.kD = TURRET.TURRET_D1; 
  
        tMotor.getConfigurator().apply(tMotorConfiguration);

        // Activate motion magic to hold turret in starting position
        // tMotor.setPosition(0.0);
        //TODO: Figure out if this is causing issues, or what is causing turret default behavior
        //tMotor.setControl(motionMagicRequest.withSlot(1).withPosition(tMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Angle", 0.0);

        // Instantiate for calculating the turret angle based on target and robot positions
        angleCalc = new AutoTurretAngle();

        // SmartDashboard.putNumber("P_TUR", TURRET.TURRET_P0);
        // SmartDashboard.putNumber("I_TUR", TURRET.TURRET_I0);
        // SmartDashboard.putNumber("D_TUR", TURRET.TURRET_D0);
        // SmartDashboard.putNumber("S_TUR", TURRET.TURRET_S);

        this.resetPos(E1_value, E2_value);
    }

    public double calcAngle(Pose2d robotPosition, Pose2d targetLocation){
        return angleCalc.TurretAngleCalc(robotPosition, targetLocation);
    }


    /**
     * Move turret to track target position
     * @param robotPosition Current position of the robot from odometry, in field coordinates
     * @param targetLocation The centerpoint of the target we are trying to track, in field coordinates
     */
    public void TurrettoAngle(Pose2d robotPosition, Pose2d targetLocation) {
        // Calculate target angle based on robot and target positions
        double targetAngle = calcAngle(robotPosition, targetLocation);

        // Turret only moves +/- 180 degrees, so adjust target angle if it is outside of that range
        if(Math.abs(targetAngle) > 180){
            if(targetAngle < 0)
                targetAngle += 360;
            else
                targetAngle -= 360;
        }
        Logger.recordOutput(TURRET.LOG_PATH + "targetAngle", targetAngle);

        // when the angle of the turret is within x degrees of the target angle, switch to less aggressive PID values in slot 1
        //TODO: Implement two types of PID if necessary
        // int currentSlot = 0;
        // if(Math.abs(tMotor.getPosition().getValueAsDouble() / TURRET.DEGREES_TO_MOTOR_ROTATIONS - targetAngle) <= TURRET.TURRET_TOLERANCE){
        //     currentSlot = 1;
        // }

        //
        // Set motor position based on target angle, converting from degrees to motor rotations
        //
        tMotor.setControl(positionRequest.withSlot(0).withPosition(targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS)); // PID Position control for testing
        //tMotor.setControl(positionTorqueCurrent.withSlot(0).withPosition(targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        //TODO: Implement Motion magic for turret
        //tMotor.setControl(motionMagicRequest.withSlot(currentSlot).withPosition(targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
    }


    /** 
     * Stop the turret motor.  Not normally used; we want the turret to hold position with the motor
     */
    public void stop(){
        tMotor.setVoltage(0);
    }

    public double getAngle(){
        return tMotor.getPosition().getValueAsDouble() * 1.0/(TURRET.DEGREES_TO_MOTOR_ROTATIONS);
    }

    /**
     * Get the encoder values the define the turret zero position.
     */
    public void resetPos(double E1, double E2) {
        System.out.println("Resetting Pose");
        tMotor.setPosition(0);
        //tMotor.setPosition(CRT(E1 - TURRET.E1_OFFSET, E2 - TURRET.E2_OFFSET) * (TURRET.TURRET_TG * 1.0) / TURRET.TURRET_GM);
    }

    /**
     * Command to move turret to track target position.
     * Must be called each time the robot moves or target changes
     * @param robotPosition Current position of the robot from odometry, in field coordinates
     * @param targetLocation The centerpoint of the target we are trying to track, in field coordinates
     */
    public Command TurrettoAngleCommand(Pose2d robotPosition, Pose2d targetLocation) {
        return this.runOnce(() -> TurrettoAngle(robotPosition, targetLocation));
    }

    /**
     * Command to stop the turret motor.  Not normally used; we want the turret to hold position with the motor
     */
    public Command stopTurretCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Command to reset the zero position of the turret.  This is used to recalibrate the turret, not for live control
     * @return
     */
    public Command resetPosCommand() {
        return this.runOnce(() -> resetPos(0, 0));
    }

    /**
     * 
     * @param E1
     * @param E2
     * @return returns value relative to main turret gear of offset necessary to recenter turret
     */
    public static double CRT(double E1, double E2){
        double R1 = E1/360.0;
        double R2 = E2/360.0;
        double G1 = (TURRET.TURRET_G1 * 1.0) / TURRET.TURRET_TG;
        double G2 = (TURRET.TURRET_G2 * 1.0) / TURRET.TURRET_TG;
        for(int i = 1; i <= TURRET.TURRET_TG; i++){
            double V1 = (i + R1) * G1 * 1.0;
            double V2 = (i + R2) * G2 * 1.0;
            double V1New = (i + 1 + R1) * G1 * 1.0;
            double V2Old = (i - 1 + R2) * G2 * 1.0;
            double V1Old = (i - 1 + R1) * G1 * 1.0;
            double V2New  = (i + 1 + R2) * G2 * 1.0;

            if(Math.abs(V1Old - V2Old) <= TURRET.TURRET_TOLERANCE){
                map.put(Math.abs(V1Old - V2Old), (V1Old + V2Old) / 2.0);
            }            
            
            if((Math.abs(V1 - V2Old) <= TURRET.TURRET_TOLERANCE)){
                map.put(Math.abs(V1 - V2Old), (V1 + V2Old) / 2.0);
            }

            if(Math.abs(V1Old - V2) <= TURRET.TURRET_TOLERANCE){
                map.put(Math.abs(V1Old - V2), (V1Old + V2) / 2.0);
            }
            
            if((Math.abs(V1 - V2) <= TURRET.TURRET_TOLERANCE)){
                map.put(Math.abs(V1 - V2), (V1 + V2) / 2.0);
            }
            if(Math.abs(V1New - V2) <= TURRET.TURRET_TOLERANCE){
                map.put(Math.abs(V1New - V2), (V1New + V2) / 2.0);
            }

            if(Math.abs(V1 - V2New) <= TURRET.TURRET_TOLERANCE){
                map.put(Math.abs(V1 - V2New), (V1 + V2New) / 2.0);
            }

            if(Math.abs(V1New - V2New) <= TURRET.TURRET_TOLERANCE){
                map.put(Math.abs(V1New - V2New), (V1New + V2New) / 2.0);
            }

            set = map.keySet();
            if(set.size() != 0){
            System.out.println("Set: " + set.toString());

            Double lowest = Collections.min(set);
            System.out.println("Lowest: " + lowest);

            return map.get(lowest);
            }
        }
        return 0;
    }

      

    @Override
    public void periodic(){
        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Angle", tMotor.getPosition().getValueAsDouble() * (1/TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput(TURRET.LOG_PATH +"Motor Rotations", tMotor.getPosition().getValueAsDouble()); //rotations per second
        Logger.recordOutput(TURRET.LOG_PATH +"Motor Voltage", tMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(TURRET.LOG_PATH + "CRT Value", CRT(E1.get() - TURRET.E1_OFFSET, E2.get() - TURRET.E2_OFFSET));
        Logger.recordOutput(TURRET.LOG_PATH + "E1 Offset", E1.get() - TURRET.E1_OFFSET);
        Logger.recordOutput(TURRET.LOG_PATH + "E2 Offset", E2.get() - TURRET.E2_OFFSET);
    }

 public void updatePID(){

        double P  = SmartDashboard.getNumber("P_TUR", TURRET.TURRET_P0);
        double I  = SmartDashboard.getNumber("I_TUR", TURRET.TURRET_I0);
        double D  = SmartDashboard.getNumber("D_TUR", TURRET.TURRET_D0);
        double S = SmartDashboard.getNumber("S_TUR", TURRET.TURRET_S);

        if(P != P_OLD || I != I_OLD || D != D_OLD || S != S_OLD){
            tMotorConfiguration.Slot0.kP = P; 
            tMotorConfiguration.Slot0.kI = I;
            tMotorConfiguration.Slot0.kD = D; 
            tMotorConfiguration.Slot0.kS = S;

            P_OLD = P;
            I_OLD = I;
            D_OLD = D;
            S_OLD = S;
            tMotor.getConfigurator().apply(tMotorConfiguration);

        }
    }

}
