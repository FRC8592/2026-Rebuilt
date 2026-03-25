package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import java.lang.Math;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import java.lang.Math;

public class Turret extends SubsystemBase {

    private TalonFX tMotor;
    private TalonFXConfiguration tMotorConfiguration;
    // private MotionMagicConfigs TurretMMConfig;
    
    // private MotionMagicExpoTorqueCurrentFOC turretMMETorqueCurrentFOC;
    // private PositionTorqueCurrentFOC positionTorqueCurrent;
    private PositionVoltage positionVoltage;

    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;

    private double P_OLD;
    private double I_OLD;
    private double D_OLD;

    private double targetAngle;


    private static Map<Double, Double> map = new HashMap<Double, Double>();
    private static Set<Double> set = new HashSet<Double>();

    public Turret() {
       
        /**
         * Instantiate Absolute Encoders necessary for CRT Calculation
         */
        E1 = new DutyCycleEncoder(0, 360, TURRET.E1_OFFSET);
        E2 = new DutyCycleEncoder(1, 360, TURRET.E2_OFFSET);



        /**
         * Initialize the Turret Motor and Turret Motor Config
         */
        tMotor = new TalonFX(TURRET.TURRET_MOTOR_CAN_ID);
        tMotorConfiguration = new TalonFXConfiguration();



        /**
         * Initialize Turret Motor Config, three to chose from
         */
        positionVoltage = new PositionVoltage(0);
        // TurretMMConfig = new MotionMagicConfigs();
        // turretMMETorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);

        //TODO: Test if this works
        //positionVoltage.withOverrideBrakeDurNeutral(true);

        /**
         * Turret Motor Inverted Value Configuration
         * ONLY LOOK AT THE VALUE OF THE INVERTEDVALUE, THAT GIVES US THE ROTATION FOR POSITIVE MOTION
         */
        tMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;



        /**
         * Turret Motor Current Limit Configuration, which limits supply current too.
         */
        //TODO: Reconfigure this
        tMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        tMotorConfiguration.CurrentLimits.StatorCurrentLimit = TURRET.TURRET_CURRENT_LIMIT;



        /**
         * Turret Motor Neutral Mode Configuration. Tells the motor what to do when at 0V
         * (I believe so)
         */
        //TODO: Check why turret was very easy to move even when motor was in brake mode
        tMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;



        /**
         * Turret Motor Software "Soft" Limit Configurations. These prevent overrotation of the turret.
         */
        tMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        tMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        tMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET.FORWARD_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS;
        tMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET.REVERSE_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS;


        
        /**
         * Turret Motor PID Configuration and Constants
         */
        tMotorConfiguration.Slot0.kP = TURRET.TURRET_P0;
        tMotorConfiguration.Slot0.kI = TURRET.TURRET_I0;
        tMotorConfiguration.Slot0.kD = TURRET.TURRET_D0;
        tMotorConfiguration.Slot0.kS = TURRET.TURRET_S;



        /**
         * Motion Magic Configuration and Constants
         */
        // TurretMMConfig.MotionMagicAcceleration = TURRET.MAX_ACCELERATION;
        // TurretMMConfig.MotionMagicJerk = TURRET.MAX_JERK;
        // TurretMMConfig.MotionMagicCruiseVelocity = TURRET.CRUISE_VELOCITY;
        //tMotorConfiguration.MotionMagic = TurretMMConfig;

        /** */
        tMotor.getConfigurator().apply(tMotorConfiguration);

        SmartDashboard.putNumber("P_TUR", 0);
        SmartDashboard.putNumber("I_TUR", 0);
        SmartDashboard.putNumber("D_TUR", 0);

        // Instantiate for calculating the turret angle based on target and robot
        // positions
        this.resetPos();
    }


    public void TurrettoAngle(Pose2d robotPosition, double angle) {
        double robotAngle = robotPosition.getRotation().getDegrees();
        double target = angle - robotAngle - TURRET.TURRET_ANGLE_OFFSET;
        if(Math.abs(target) > 180){
            if(target < 0)
                target += 360;
            else
                target -= 360;
        }
        targetAngle = target;
        tMotor.setControl(positionVoltage.withSlot(0).withPosition(target * TURRET.DEGREES_TO_MOTOR_ROTATIONS)); // PID Position control for testing
    }
    /**
     * Stop the turret motor. Not normally used; we want the turret to hold position with the motor
     */
    public void stop() {
        tMotor.setVoltage(0);
    }

    public double getAngle() {
        return tMotor.getPosition().getValueAsDouble() * 1.0 / (TURRET.DEGREES_TO_MOTOR_ROTATIONS);
    }

    public void holdPosition() {
        double holdPos = tMotor.getPosition().getValueAsDouble();
        tMotor.setControl(positionVoltage.withSlot(0).withPosition(holdPos));
    }

    public void basicTurretToPos(double angle){
        targetAngle = angle;
        tMotor.setControl(positionVoltage.withSlot(0).withPosition(angle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
    }

    public Command basicTurretToPosCommand(double angle){
        return this.runOnce(() -> basicTurretToPos(angle));
    }

    /**
     * Get the encoder values the define the turret zero position.
     */
    public void resetPos() {
        System.out.println("Resetting Pose");
        tMotor.setPosition(0);
        // tMotor.setPosition(CRTTypeTwo(E1.get() - TURRET.E1_OFFSET, E2.get() -
        // TURRET.E2_OFFSET) * 96.0 / 10.0);
        // System.out.println("CRT Raw Value: " + CRTTypeTwo(E1.get(), E2.get()));
        // System.out.println("CRT Rotations " + CRTTypeTwo(E1.get(), E2.get()) * 96.0 /
        // 10.0);
        // To make sure this works!
        // tMotor.setPosition(0);
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    /**
     * Command to move turret to track target position. Must be called each time the robot moves or
     * target changes
     * 
     * @param robotPosition Current position of the robot from odometry, in field coordinates
     * @param targetLocation The centerpoint of the target we are trying to track, in field
     *        coordinates
     */
    // public Command TurrettoAngleCommand(Pose2d robotPosition, Pose2d targetLocation) {
    //     return this.runOnce(() -> TurrettoAngle(robotPosition, targetLocation));
    // }

    /**
     * Command to stop the turret motor. Not normally used; we want the turret to hold position with
     * the motor
     */
    public Command stopTurretCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Command to reset the zero position of the turret. This is used to recalibrate the turret, not
     * for live control
     * 
     * @return
     */
    public Command resetPosCommand() {
        return this.runOnce(() -> resetPos());
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

            if(Math.abs(V1Old - V2Old) <= TURRET.CRT_TOLERANCE){
                map.put(Math.abs(V1Old - V2Old), (V1Old + V2Old) / 2.0);
            }            
            
            if((Math.abs(V1 - V2Old) <= TURRET.CRT_TOLERANCE)){
                map.put(Math.abs(V1 - V2Old), (V1 + V2Old) / 2.0);
            }

            if(Math.abs(V1Old - V2) <= TURRET.CRT_TOLERANCE){
                map.put(Math.abs(V1Old - V2), (V1Old + V2) / 2.0);
            }
            
            if((Math.abs(V1 - V2) <= TURRET.CRT_TOLERANCE)){
                map.put(Math.abs(V1 - V2), (V1 + V2) / 2.0);
            }
            if(Math.abs(V1New - V2) <= TURRET.CRT_TOLERANCE){
                map.put(Math.abs(V1New - V2), (V1New + V2) / 2.0);
            }

            if(Math.abs(V1 - V2New) <= TURRET.CRT_TOLERANCE){
                map.put(Math.abs(V1 - V2New), (V1 + V2New) / 2.0);
            }

            if(Math.abs(V1New - V2New) <= TURRET.CRT_TOLERANCE){
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
    public void periodic() {


        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());

        //TODO: Implement CRT Again!
        // Logger.recordOutput("Gear Ticks " , CRT(E1.get(), E2.get() -
        // ));
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Angle",
                tMotor.getPosition().getValueAsDouble() * (1 / TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Rotations",
                tMotor.getPosition().getValueAsDouble()); // rotations
                                                          // per second
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Voltage",
                tMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(TURRET.LOG_PATH + "Turret Target Angle", targetAngle);

    }

    public void updatePID() {

        double P = SmartDashboard.getNumber("P_TUR", TURRET.TURRET_P0);
        double I = SmartDashboard.getNumber("I_TUR", TURRET.TURRET_I0);
        double D = SmartDashboard.getNumber("D_TUR", TURRET.TURRET_D0);

        if (P != P_OLD || I != I_OLD || D != D_OLD) {
            tMotorConfiguration.Slot0.kP = P;
            tMotorConfiguration.Slot0.kI = I;
            tMotorConfiguration.Slot0.kD = D;

            P_OLD = P;
            I_OLD = I;
            D_OLD = D;


            tMotor.getConfigurator().apply(tMotorConfiguration);

        }
    }

}
