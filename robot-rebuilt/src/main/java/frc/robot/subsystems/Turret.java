package frc.robot.subsystems;

import frc.robot.helpers.motor.talonfx.KrakenX44Motor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;

import java.lang.Math;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class Turret extends SubsystemBase{
    private KrakenX44Motor tMotor;
    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;
    private DutyCycleOut percentOutput;

    public Turret(){
        E1 = new DutyCycleEncoder(0, 1, 0);
        E2 = new DutyCycleEncoder(1, 1, 0);
        tMotor = new KrakenX44Motor(19 ,true);
        PIDProfile gains = new PIDProfile();
        gains.setPID(TURRET.TURRET_P, TURRET.TURRET_I, TURRET.TURRET_D);
        //tMotor.withGains(gains);
        //tMotor.configureMotionMagic(20, 4);
    }
    public void TurrettoPos(double position){
        System.out.println("Going into positioning method");
        tMotor.setPercentOutput(0.05);
    }
    public void stop(){
        tMotor.setPercentOutput(0);
    }

    public Command TurrettoPosCommand(double position){
        System.out.println("Going into positioning command");
        return this.runOnce(() -> TurrettoPos(position));
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

    public static double CRTTypeOne(double E1, double E2){
        int R1 = (int)(E1 * TURRET.TURRET_G1); 
        int R2 = (int)(E2 * TURRET.TURRET_G2);
        // double R1Filtered = R1 / 10.0;
        // double R2Filtered = R2 / 10.0;

        System.out.println("R1: " + R1 + " R2: " + R2);
        int M1 = TURRET.TURRET_TOTAL / TURRET.TURRET_G1;
        int M2 = TURRET.TURRET_TOTAL / TURRET.TURRET_G2;
        // int M1Total = 1;
        // int M2Total = 1;
        // while(M1Total % M1 != 0){
        //     M1Total += TURRET.TURRET_G1;
        // }
        // M1Inverse = M1Total/M1;
        // while(M2Total % M2 != 0){
        //     M2Total += TURRET.TURRET_G2;
        // }
        // M2Inverse = M2Total/M2;
        double GearRotation = (R1 * M1 * 1 + R2* M2 * 10) % TURRET.TURRET_TG;
        return GearRotation; 
    }

    // @Override
    // public void periodic(){
    //     double E1Raw = E1.get();
    //     double E2Raw = E2.get();
    //     int E1Process = (int)(E1Raw * 1000);
    //     int E2Process = (int)(E2Raw * 1000);
    //     double E1Filter = E1Process / 1000.0;
    //     double E2Filter = E2Process / 1000.0;
    //     System.out.println("E1: " + E1Filter + " E2: " + E2Filter);
    //     //System.out.println("Degrees: " + CRTTypeOne(E1Filter, E2Filter) / TURRET.TURRET_TG * 360);
    //     SmartDashboard.putNumber("Gear Ticks", CRTTypeOne(E1Filter, E2Filter));
    //     System.out.println("Motor Position: " + tMotor.getRotations());
    // }
}
