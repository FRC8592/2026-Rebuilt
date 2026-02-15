package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.AutoTurretAngle;
import frc.robot.subsystems.swerve.Swerve;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.lang.Math;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Turret extends SubsystemBase{
    private TalonFX tMotor;
    private DutyCycleEncoder E1;
    private DutyCycleEncoder E2;
    private double AOriginal = TURRET.INITIAL_MAX_ACCELERATION;
    private double VOriginal = TURRET.INITIAL_CRUISE_VELOCITY;
    private AutoTurretAngle AngleCalc;
    private Swerve swerve;
    private TalonFXConfiguration tMotorConfiguration; 

    private MotionMagicConfigs motionMagicConfig;

    private PositionVoltage positionOutput;

    private MotionMagicVoltage motionMagicOutput;

    private VoltageOut voltageOutput;

    public Turret(Swerve swerve){
        this.swerve = swerve;
        E1 = new DutyCycleEncoder(0, 360, 0);
        E2 = new DutyCycleEncoder(1, 360, 0);
        tMotor = new TalonFX(TURRET.TURRET_MOTOR);
        tMotorConfiguration = new TalonFXConfiguration();
        tMotorConfiguration.Slot0.kP = TURRET.TURRET_P; 
        tMotorConfiguration.Slot0.kI = TURRET.TURRET_I;
        tMotorConfiguration.Slot0.kD = TURRET.TURRET_D;
        positionOutput = new PositionVoltage(0.0);

        //TODO: Change this to utilize Motion Magic Expo instead for better flywheel adjustment
        motionMagicConfig = tMotorConfiguration.MotionMagic;
        motionMagicConfig.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(AOriginal))
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(VOriginal))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100 * AOriginal));
        motionMagicOutput = new MotionMagicVoltage(0);
        tMotor.getConfigurator().apply(tMotorConfiguration);

        SmartDashboard.putNumber("Acceleration", 80);
        SmartDashboard.putNumber("CruiseVelocity", 6);
        SmartDashboard.putNumber("Position Value", 0);

        AngleCalc = new AutoTurretAngle(this.swerve);
    }
    public void TurrettoPos(Pose2d targetLocation){
        double initialPos = AngleCalc.TurretAngleCalc(swerve.getCurrentOdometryPosition(), targetLocation);
        if(Math.abs(AngleCalc.rawAngle) > 180){
            if(AngleCalc.rawAngle < 0)
                initialPos+=360;
            else
                initialPos-=360;
        }
        //tMotor.setPosition(initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
        Logger.recordOutput("Motor Set Position", initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
        System.out.println("Position: " + initialPos/360);
        //tMotor.setControl(positionOutput.withSlot(0).withPosition(initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS));

        //TODO: Implement Motion Magic
        tMotor.setControl(motionMagicOutput.withSlot(0).withPosition(initialPos * TURRET.DEGREES_TO_MOTOR_ROTATIONS));
    }
    public void stop(){
        tMotor.setControl(voltageOutput.withOutput(0));
    }

    public Command TurrettoPosCommand(Pose2d targetLocation){
        return this.run(() -> TurrettoPos(targetLocation));
    }

    public Command stopTurretCommand(){
        return this.runOnce(() -> stop());
    }

    public Command resetPosCommand(){
        return this.runOnce(() -> resetPos());
    }

    public void resetPos(){
        System.out.println("Resetting Pose");
        tMotor.setPosition(CRTTypeTwo(E1.get() - TURRET.E1_OFFSET, E2.get() - TURRET.E2_OFFSET) * TURRET.TG / TURRET.GM);
        System.out.println("CRT Raw Value: " + CRTTypeTwo(E1.get(), E2.get()));
        System.out.println("CRT Rotations " + CRTTypeTwo(E1.get(), E2.get()) * TURRET.TG / TURRET.GM);
    }

    public void updateMotionMagic(){
        double acceleration = SmartDashboard.getNumber("Acceleration", 80);
        double cruiseVelocity = SmartDashboard.getNumber("CruiseVelocity", 6);
        if(acceleration != AOriginal || cruiseVelocity != VOriginal){
            motionMagicConfig.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(acceleration))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(cruiseVelocity))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100 * acceleration));
            AOriginal = acceleration;
            VOriginal = cruiseVelocity;
        }
    }

    public static double CRTTypeTwo(double E1, double E2){
        double R1 = E1 * TURRET.TO_ROTATIONS;
        double R2 = E2 * TURRET.TO_ROTATIONS;
        double G1 = TURRET.G1/TURRET.TG;
        double G2 = TURRET.G2/TURRET.TG;
        for(int i = 0; i <= (int)(TURRET.TG); i++){
            double V1 = (i + R1) * G1;
            double V2 = (i + R2) * G2;
            double V1New = (i + 1 + R1) * G1;
            double V2Old = (i - 1 + R2) * G2;
            if((Math.abs(V1 - V2) <= TURRET.CRT_TOLERANCE) || (Math.abs(V1 - V2Old) <= TURRET.CRT_TOLERANCE)){
                return V1;
            }
            if(Math.abs(V1New - V2) <= TURRET.CRT_TOLERANCE){
                return V2;
            }
        }
        return 0;
    }

      

    @Override
    public void periodic(){
        double E1R = E1.get();
        double E2R = E2.get();
        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());
        //Logger.recordOutput("R1", ((int)(E1Filter * TURRET.TURRET_G1)));
        //Logger.recordOutput("R2", ((int)(E2Filter * TURRET.TURRET_G2)));
        //Logger.rec("E1: " + E1Filter + " E2: " + E2Filter);
        Logger.recordOutput("Gear Ticks " , CRTTypeTwo(E1R - TURRET.E1_OFFSET, E2R - TURRET.E2_OFFSET));
        //Logger.recordOutput("Motor Angle", tMotor.getRotations() * (1/TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput("Motor Rotations", tMotor.getPosition().getValueAsDouble()); //rotations per second
        //updateMotionMagic();
    }
}
