package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import java.lang.Math;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import static edu.wpi.first.units.Units.*;


public class Turret extends SubsystemBase {

    private TalonFX turretMotor;
    private TalonFXConfiguration turretMotorConfig;
    //private MotionMagicConfigs turretMMConfig;
    private MotorOutputConfigs turretMotorOutputConfig;
    private SoftwareLimitSwitchConfigs positionLimitConfig;
    private Slot0Configs turretPIDConfig;


    private PositionVoltage positionRequest;
    //private MotionMagicVoltage positionMMRequest;



    //TODO: Remove final keyword if necessary
    private final DutyCycleEncoder E1;
    private final DutyCycleEncoder E2;

    private double P_SET;
    private double I_SET;
    private double D_SET;

    private double targetAngle;

    private static Map<Double, Double> mapValues = new HashMap<Double, Double>();
    private static Set<Double> setLeast = new HashSet<Double>();

    public Turret() {
       
        /**
         * Instantiate Absolute Encoders necessary for CRT Calculation
         */
        E1 = new DutyCycleEncoder(0, 360, TURRET.E1_OFFSET.in(Degrees));
        E2 = new DutyCycleEncoder(1, 360, TURRET.E2_OFFSET.in(Degrees));



        /**
         * Initialize the Turret Motor and Turret Motor Config
         */
        turretMotor = new TalonFX(TURRET.TURRET_MOTOR_CAN_ID);
        turretMotorConfig = new TalonFXConfiguration();
        turretMotorOutputConfig = new MotorOutputConfigs();
        positionLimitConfig = new SoftwareLimitSwitchConfigs();
        turretPIDConfig = new Slot0Configs();
        //turretMMConfig = new MotionMagicConfigs();




        /**
         * Initialize Turret Motor Control Modes, two control modes to choose from
         */
        //TODO: First utilize positionVoltage then switch to MMVoltage
        positionRequest = new PositionVoltage(0);
        // positionMMRequest = new MotionMagicVoltage(0);

        //TODO: Test if this works
        positionRequest.withOverrideBrakeDurNeutral(true);
        // positionMMRequest.withOverrideBrakeDurNeutral(true);


        turretMotorOutputConfig.withInverted(InvertedValue.Clockwise_Positive);



        /**
         * Turret Motor Current Limit Configuration, which limits supply current too.
         */
        //TODO: Reconfigure this
        turretMotorConfig.CurrentLimits.withStatorCurrentLimit(TURRET.CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);

        //TODO: Test this if PID Tuning prevents stalling, and IF and ONLY IF we utilize OverrideBrakeDurNeutral
        //tMotorOutputConfig.withDutyCycleNeutralDeadband(0.01);


        /**
         * Turret Motor Neutral Mode Configuration. Tells the motor what to do when at 0V
         * (I believe so)
         */
        //TODO: Check why turret was very easy to move even when motor was in brake mode
        turretMotorOutputConfig.withNeutralMode(NeutralModeValue.Brake);

        turretMotorConfig.withMotorOutput(turretMotorOutputConfig);


        /**
         * Turret Motor Software "Soft" Limit Configurations. These prevent overrotation of the turret.
         */

        positionLimitConfig.withForwardSoftLimitThreshold(TURRET.FORWARD_LIMIT.in(Degrees) * TURRET.DEGREES_TO_MOTOR_ROTATIONS)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(TURRET.REVERSE_LIMIT.in(Degrees) * TURRET.DEGREES_TO_MOTOR_ROTATIONS)
        .withReverseSoftLimitEnable(true);


        turretMotorConfig.withSoftwareLimitSwitch(positionLimitConfig);

        
        /**
         * Turret Motor PID Configuration and Constants
         */
        turretPIDConfig
        .withKP(TURRET.TURRET_P.in(Volts))
        .withKI(TURRET.TURRET_I.in(Volts))
        .withKD(TURRET.TURRET_D.in(Volts))
        .withKS(TURRET.TURRET_S.in(Volts))
        .withKV(TURRET.TURRET_V.in(Volts))
        .withKA(TURRET.TURRET_A.in(Volts));

        turretMotorConfig.withSlot0(turretPIDConfig);


        /**
         * Motion Magic Configuration and Constants
         */
        // turretMMConfig.MotionMagicAcceleration = TURRET.MAX_ACCELERATION.in(RotationsPerSecondPerSecond);
        // turretMMConfig.MotionMagicJerk = TURRET.MAX_JERK.in(RotationsPerSecondPerSecond.per(Second));
        // turretMMConfig.MotionMagicCruiseVelocity = TURRET.CRUISE_VELOCITY.in(RotationsPerSecond);
        // turretMotorConfig.withMotionMagic(turretMMConfig);

        /** */
        turretMotor.getConfigurator().apply(turretMotorConfig);

        SmartDashboard.putNumber("tP", TURRET.TURRET_P.in(Volts));
        SmartDashboard.putNumber("tI", TURRET.TURRET_I.in(Volts));
        SmartDashboard.putNumber("tD", TURRET.TURRET_D.in(Volts));

        // Instantiate for calculating the turret angle based on target and robot
        // positions
        this.resetPos();
    }


    public void TurrettoAngle(Pose2d robotPosition, double angle) {
        double robotAngle = robotPosition.getRotation().getDegrees();
        double target = angle - robotAngle - TURRET.TURRET_ANGLE_OFFSET.in(Degrees);
        if(Math.abs(target) > TURRET.MAX_ROTATION_LIMIT.in(Degrees))
            target -= (int)(Math.signum(target)) * 360d;
        logAngle(target);
        turretMotor.setControl(positionRequest.withSlot(0).withPosition(target * TURRET.DEGREES_TO_MOTOR_ROTATIONS)); // PID Position control for testing
        //turretMotor.setControl(positionMMRequest.withPosition(target * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
    }

    public void basicTurretToPos(double angle){
        logAngle(angle);
        turretMotor.setControl(positionRequest.withSlot(0).withPosition(angle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        //turretMotor.setControl(positionMMRequest.withPosition(angle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
    }

    public Command basicTurretToPosCommand(double angle){
        return this.runOnce(() -> basicTurretToPos(angle));
    }
    /**
     * Stop the turret motor. Not normally used; we want the turret to hold position with the motor
     */
    public void stop() {
        turretMotor.setVoltage(0);
    }

    /**
     * Command to stop the turret motor. Not normally used; we want the turret to hold position with
     * the motor
     */
    public Command stopTurretCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Get the encoder values the define the turret zero position.
     */
    public void resetPos() {
        turretMotor.setPosition(0);
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

    public void holdPosition() {
        double holdPos = turretMotor.getPosition().getValueAsDouble();
        turretMotor.setControl(positionRequest.withSlot(0).withPosition(holdPos));
        //turretMotor.setControl(positionMMRequest.withPosition(holdPos));
    }

    /**
     * 
     * @param E1
     * @param E2
     * @return returns value relative to main turret gear of offset necessary to recenter turret
     */
     public static double calcAngle(double E1, double E2){
        double R1 = E1/360d;
        double R2 = E2/360d;
        double G1 = (TURRET.TURRET_G1 * 1.0) / TURRET.TURRET_GT;
        double G2 = (TURRET.TURRET_G2 * 1.0) / TURRET.TURRET_GT;
        for(int i = 1; i <= TURRET.TURRET_GT; i++){
            double V1 = (i + R1) * G1;
            double V2 = (i + R2) * G2 * 1.0;
            double V1New = (i + 1 + R1) * G1 * 1.0;
            double V2Old = (i - 1 + R2) * G2 * 1.0;
            double V1Old = (i - 1 + R1) * G1 * 1.0;
            double V2New  = (i + 1 + R2) * G2 * 1.0;

            if(Math.abs(V1Old - V2Old) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1Old - V2Old), (V1Old + V2Old) / 2d);
            }            
            
            if(Math.abs(V1 - V2Old) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1 - V2Old), (V1 + V2Old) / 2d);
            }

            if(Math.abs(V1Old - V2) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1Old - V2), (V1Old + V2) / 2d);
            }
            
            if(Math.abs(V1 - V2) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1 - V2), (V1 + V2) / 2d);
            }
            if(Math.abs(V1New - V2) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1New - V2), (V1New + V2) / 2d);
            }

            if(Math.abs(V1 - V2New) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1 - V2New), (V1 + V2New) / 2d);
            }

            if(Math.abs(V1New - V2New) <= TURRET.CRT_TOLERANCE.in(Rotations)){
                mapValues.put(Math.abs(V1New - V2New), (V1New + V2New) / 2d);
            }

            setLeast = mapValues.keySet();
            if(setLeast.size() != 0){
            System.out.println("Set: " + setLeast.toString());

            Double lowest = Collections.min(setLeast);
            System.out.println("Lowest: " + lowest);

            return mapValues.get(lowest);
            }
        }
        return 0;
    }

    public double getAngle() {
        return turretMotor.getPosition().getValueAsDouble() / (TURRET.DEGREES_TO_MOTOR_ROTATIONS);
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    public void logAngle(double angle){
        targetAngle = angle;
    }




    @Override
    public void periodic() {


        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());

        //TODO: Implement CRT Again!
        // Logger.recordOutput("Gear Ticks " , CRT(E1.get(), E2.get() -
        // ));
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Angle",
                turretMotor.getPosition().getValueAsDouble() * (1 / TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Rotations",
                turretMotor.getPosition().getValueAsDouble()); // rotations
                                                          // per second
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Voltage",
                turretMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(TURRET.LOG_PATH + "Turret Target Angle", targetAngle);

    }

     public void updatePID() {

        //Receive Turret PID Constants from SmartDashboard
        
        double P_NEW = SmartDashboard.getNumber("tP", TURRET.TURRET_P.in(Volts));
        double I_NEW = SmartDashboard.getNumber("tI", TURRET.TURRET_I.in(Volts));
        double D_NEW = SmartDashboard.getNumber("tD", TURRET.TURRET_D.in(Volts));

        boolean tDiff = (P_SET != P_NEW || I_SET != I_NEW || D_SET != D_NEW);


       if(tDiff){
        turretPIDConfig
        .withKP(P_NEW)
        .withKI(I_NEW)
        .withKD(D_NEW);
        turretMotorConfig.withSlot0(turretPIDConfig);
        turretMotor.getConfigurator().apply(turretMotorConfig);

        P_SET = P_NEW;
        I_SET = I_NEW;
        D_SET = D_NEW;
       }

    }


}
