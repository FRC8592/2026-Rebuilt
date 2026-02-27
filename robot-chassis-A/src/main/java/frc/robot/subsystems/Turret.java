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
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import java.lang.Math;

//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Turret extends SubsystemBase{
    // Motor for turret rotation
    private TalonFX tMotor;
    private TalonFXConfiguration tMotorConfiguration;
    private PositionVoltage positionRequest;
    private VelocityVoltage spinMotorSlot0VelocityRequest = new VelocityVoltage(0);
    private MotionMagicVoltage motionMagicRequest;
    // Absolute encoders used to find the starting position of the turret
    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;
    private double E1_value;
    private double E2_value;
    // Calculate turret angle based on a target location and the robot's current position
    private AutoTurretAngle angleCalc;

    private double P_OLD;
    private double I_OLD;
    private double D_OLD;

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
        motionMagicRequest = new MotionMagicVoltage(0);

        // Put motor in brake mode and apply current limits
        tMotor.setNeutralMode(NeutralModeValue.Brake);
        tMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        tMotorConfiguration.CurrentLimits.StatorCurrentLimit = TURRET.TURRET_CURRENT_LIMIT;
        tMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply soft limits to help avoid driving the turret past the cable extension
        // ToDO: Enable and configure soft limits

        // Configure PID controls and Motion Magic parameters
        tMotorConfiguration.Slot0.kP = TURRET.TURRET_P; 
        tMotorConfiguration.Slot0.kI = TURRET.TURRET_I;
        tMotorConfiguration.Slot0.kD = TURRET.TURRET_D; 
        tMotorConfiguration.MotionMagic.MotionMagicAcceleration = TURRET.MAX_ACCELERATION;
        tMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = TURRET.CRUISE_VELOCITY;
        tMotorConfiguration.MotionMagic.MotionMagicJerk = TURRET.MAX_JERK;
        tMotorConfiguration.ClosedLoopGeneral.GainSchedErrorThreshold = 0.5; // TODO: Understand this parameter or delete!
  
        tMotor.getConfigurator().apply(tMotorConfiguration);

        //
        // TODO: Remove this? 
        // Activate motion magic to hold turret in starting position\
        //
        tMotor.setControl(motionMagicRequest.withSlot(0).withPosition(tMotor.getPosition().getValueAsDouble()));

        // Instantiate for calculating the turret angle based on target and robot positions
        angleCalc = new AutoTurretAngle();

        SmartDashboard.putNumber("P_TUR", TURRET.TURRET_P);
        SmartDashboard.putNumber("I_TUR", TURRET.TURRET_I);
        SmartDashboard.putNumber("D_TUR", TURRET.TURRET_D);
    }


    /**
     * Move turret to track target position
     * @param robotPosition Current position of the robot from odometry, in field coordinates
     * @param targetLocation The centerpoint of the target we are trying to track, in field coordinates
     */
    public void TurrettoAngle(Pose2d robotPosition, Pose2d targetLocation) {
        //
        // Calculate target angle based on robot and target positions
        //
        double targetAngle = angleCalc.TurretAngleCalc(robotPosition, targetLocation);
        
        //
        // Turret only moves +/- 180 degrees, so adjust target angle if it is outside of that range
        //
        if(Math.abs(targetAngle) > 180){
            if(targetAngle < 0)
                targetAngle += 360;
            else
                targetAngle -= 360;
        }

        //
        // Set motor position based on target angle, converting from degrees to motor rotations
        //
        // tMotor.setControl(positionRequest.withSlot(0).withPosition(targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS)); // PID Position control for testing
        tMotor.setControl(motionMagicRequest.withSlot(0).withPosition(targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput("Motor Set Position", targetAngle * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
    }


    /** 
     * Stop the turret motor.  Not normally used; we want the turret to hold position with the motor
     */
    public void stop(){
        tMotor.setVoltage(0);
    }


    /**
     * Get the encoder values the define the turret zero position.
     */
    public void resetPos() {
        System.out.println("Resetting Pose");
        tMotor.setPosition(0);
        //tMotor.setPosition(CRTTypeTwo(E1.get() - TURRET.E1_OFFSET, E2.get() - TURRET.E2_OFFSET) * 96.0 / 10.0);
        //System.out.println("CRT Raw Value: " + CRTTypeTwo(E1.get(), E2.get()));
        //System.out.println("CRT Rotations " + CRTTypeTwo(E1.get(), E2.get()) * 96.0 / 10.0);
        //To make sure this works!
        //tMotor.setPosition(0);
    }

    /**
     * Command to move turret to track target position.
     * Must be called each time the robot moves or target changes
     * @param robotPosition Current position of the robot from odometry, in field coordinates
     * @param targetLocation The centerpoint of the target we are trying to track, in field coordinates
     */

     // TODO: angle parameter is for simple initial testing.  Remove.
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
        return this.runOnce(() -> resetPos());
    }

    // public void updateMotionMagic(){
    //     double acceleration = SmartDashboard.getNumber("Acceleration", 80);
    //     double cruiseVelocity = SmartDashboard.getNumber("CruiseVelocity", 6);
    //     if(acceleration != AOriginal || cruiseVelocity != VOriginal){
    //         tMotor.configureMotionMagic(acceleration, cruiseVelocity);
    //         AOriginal = acceleration;
    //         VOriginal = cruiseVelocity;
    //     }
    // }


    /**
     * 
     * @param E1
     * @param E2
     * @return returns value relative to main turret gear of offset necessary to recenter turret
     */
    public static double CRTTypeTwo(double E1, double E2){
        double R1 = E1/360.0;
        double R2 = E2/360.0;
        double G1 = 10.0/96;
        double G2 = 11.0/96;
        // double[] Encoder1Val = new double[97];
        // double[] Encoder2Val = new double[97];
        for(int i = 0; i <= 96; i++){
            double V1 = (i + R1) * G1;
            double V2 = (i + R2) * G2;
            double V1New = (i + 1 + R1) * G1;
            double V2Old = (i - 1 + R2) * G2;
            // int V1Process = (int)(V1 * 1000);
            // int V2Process = (int)(V2 * 1000);
            // double V1Filter = V1Process / 1000.0;
            // double V2Filter = V2Process / 1000.0;
            // Encoder1Val[i] = V1;
            // Encoder2Val[i] = V2;
            //Logger.recordOutput("V1Filtered", V1Filter);
            //Logger.recordOutput("V2Filtered", V2Filter);
            //System.out.println("Value 1: " + V1Filter);
            //System.out.println("Value 2: " + V2Filter);
            if((Math.abs(V1 - V2) <= 0.006) || (Math.abs(V1 - V2Old) <= 0.006)){
                //System.out.println("Going in and I Value: " + i);
                return V1;
            }
            if(Math.abs(V1New - V2) <= 0.006){
                return V2;
            }
        }
        return 0;
    }

      

    @Override
    public void periodic(){
        // double E1R = E1.get();
        // double E2R = E2.get();
        // int E1Process = (int)(E1Raw * 1000);
        // int E2Process = (int)(E2Raw * 1000);
        // double E1Filter = E1Process / 1000.0;
        // double E2Filter = E2Process / 1000.0;
        // Logger.recordOutput("E1", E1.get());
        // Logger.recordOutput("E2", E2.get());
        // Logger.recordOutput("R1", ((int)(E1Filter * TURRET.TURRET_G1)));
        // Logger.recordOutput("R2", ((int)(E2Filter * TURRET.TURRET_G2)));
        // Logger.rec("E1: " + E1Filter + " E2: " + E2Filter);
        // Logger.recordOutput("Gear Ticks " , CRTTypeTwo(E1R - TURRET.E1_OFFSET, E2R - TURRET.E2_OFFSET));
        Logger.recordOutput("Motor Angle", tMotor.getPosition().getValueAsDouble() * (1/TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput("Motor Rotations", tMotor.getPosition().getValueAsDouble()); //rotations per second
        // updateMotionMagic();
    }

 public void updatePID(){

        double P  = SmartDashboard.getNumber("P_TUR", TURRET.TURRET_P);
        double I  = SmartDashboard.getNumber("I_TUR", TURRET.TURRET_I);
        double D  = SmartDashboard.getNumber("D_TUR", TURRET.TURRET_D);

        if(P != P_OLD || I != I_OLD || D != D_OLD){
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
