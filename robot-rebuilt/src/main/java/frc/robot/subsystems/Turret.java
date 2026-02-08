package frc.robot.subsystems;

import frc.robot.helpers.motor.talonfx.KrakenX44Motor;
import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.AutoTurretAngle;
import frc.robot.subsystems.swerve.Swerve;

import java.lang.Math;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class Turret extends SubsystemBase{
    private KrakenX44Motor tMotor;
    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;
    private double AOriginal = TURRET.INITIAL_MAX_ACCELERATION;
    private double VOriginal = TURRET.INITIAL_CRUISE_VELOCITY;
    private AutoTurretAngle AngleCalc;
    private Swerve swerve;
    private PIDProfile gains;

    public Turret(Swerve swerve){
        this.swerve = swerve;
        E1 = new DutyCycleEncoder(0, 360, 0);
        E2 = new DutyCycleEncoder(1, 360, 0);
        gains = new PIDProfile();
        tMotor = new KrakenX44Motor(TURRET.TURRET_MOTOR ,true);
        gains.setPID(TURRET.TURRET_P, TURRET.TURRET_I, TURRET.TURRET_D);
        gains.setV(TURRET.TURRET_V);
        tMotor.withGains(gains);
        SmartDashboard.putNumber("Acceleration", 80);
        SmartDashboard.putNumber("CruiseVelocity", 6);
        SmartDashboard.putNumber("Position Value", 0);

        //tMotor.configureMotionMagic(80, 6);
        AngleCalc = new AutoTurretAngle(this.swerve);
    }
    public void TurrettoPos(){
        double initialPos = AngleCalc.rawAngle;
        if(Math.abs(AngleCalc.rawAngle) > 180){
            if(AngleCalc.rawAngle < 0)
                initialPos+=360;
            else
                initialPos-=360;
        }
        tMotor.setPosition(initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
        Logger.recordOutput("Motor Set Position", initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
        System.out.println("Position: " + initialPos/360);
        //tMotor.setPosition(position);
    }
    public void stop(){
        tMotor.setPercentOutput(0);
    }

    public Command TurrettoPosCommand(){
        return this.run(() -> TurrettoPos());
    }

    public Command stopTurretCommand(){
        return this.runOnce(() -> stop());
    }

    public Command resetPosCommand(){
        return this.runOnce(() -> resetPos());
    }

    public void resetPos(){
        tMotor.resetEncoderPosition(0);
    }

    public void updateMotionMagic(){
        double acceleration = SmartDashboard.getNumber("Acceleration", 80);
        double cruiseVelocity = SmartDashboard.getNumber("CruiseVelocity", 6);
        if(acceleration != AOriginal || cruiseVelocity != VOriginal){
            tMotor.configureMotionMagic(acceleration, cruiseVelocity);
            AOriginal = acceleration;
            VOriginal = cruiseVelocity;
        }
    }

    // public double CRTTypeOne(double E1, double E2){
    //     int R1 = (int)(E1 * TURRET.TURRET_G1); 
    //     int R2 = (int)(E2 * TURRET.TURRET_G2);
    //     Logger.recordOutput("R1Filtered", R1);
    //     Logger.recordOutput("R2Filtered", R2);
    //     double RawRotation = (R1 * 11 * 1.0 + R2 * 10 * 10.0);
    //     Logger.recordOutput("Raw Calculation", RawRotation);
    //     //System.out.println("R1: " + R1 + " R2: " + R2);
    //     int M1 = TURRET.TURRET_TOTAL / TURRET.TURRET_G1;
    //     int M2 = TURRET.TURRET_TOTAL / TURRET.TURRET_G2;
    //     // int M1Total = 1;
    //     // int M2Total = 1;
    //     // while(M1Total % M1 != 0){
    //     //     M1Total += TURRET.TURRET_G1;
    //     // }
    //     // M1Inverse = M1Total/M1;
    //     // while(M2Total % M2 != 0){
    //     //     M2Total += TURRET.TURRET_G2
    //     // }
    //     // M2Inverse = M2Total/M2;
    //     //System.out.println("Term 1: " + R1 * 11 * 1.0);
    //     double GearRotation = (R1 * M1 * 1.0 + R2 * M2 * 10.0) % TURRET.TURRET_TOTAL;
    //     return GearRotation; 
    // }

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
        double E1R = E1.get();
        double E2R = E2.get();
        // int E1Process = (int)(E1Raw * 1000);
        // int E2Process = (int)(E2Raw * 1000);
        // double E1Filter = E1Process / 1000.0;
        // double E2Filter = E2Process / 1000.0;
        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());
        //Logger.recordOutput("R1", ((int)(E1Filter * TURRET.TURRET_G1)));
        //Logger.recordOutput("R2", ((int)(E2Filter * TURRET.TURRET_G2)));
        //Logger.rec("E1: " + E1Filter + " E2: " + E2Filter);
        //Logger.recordOutput("Gear Ticks " , CRTTypeTwo(E1R - 146.2, E2R - 142.6));
        Logger.recordOutput("Motor Angle", tMotor.getRotations() * (1/TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        //updateMotionMagic();
    }
}
