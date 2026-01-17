package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.helpers.motor.spark.SparkFlexMotor;
import frc.robot.helpers.PIDProfile;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex.*;
import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;   
import frc.robot.Constants.*;
// import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.ThisIsSparkFlexMotor;
// import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;

public class Intake extends SubsystemBase{
    // Drive intake wheels
    // private SparkFlex intakeMotorSide;
    // private SparkFlex intakeMotorBottom;
    // Lift to deploy and stow intake
    private double MOTOR_P = 1/2900;
    private double MOTOR_I = 0.0;
    private double MOTOR_D = 0.0;
    private SparkFlexMotor intakeMotor;
    private PIDProfile tuningConstants;

   public Intake() {
        SmartDashboard.putNumber("Motor P Value", MOTOR_P);
        SmartDashboard.putNumber("Motor I Value", MOTOR_I);
        SmartDashboard.putNumber("Motor D Value", MOTOR_D);
        // Create the intake wheel and lift motors
        intakeMotor = new SparkFlexMotor(CAN.INTAKE_MOTOR_SIDE_CAN_ID, false);
        updatePID();

   }

   public void runIntakeAtSpeed(double desiredRPM){
    intakeMotor.setVelocity(desiredRPM);
   }

   public Command runIntakeCommand(double desiredRPM){
    return this.runOnce(() -> runIntakeAtSpeed(desiredRPM));
   }

   public void updatePID(){
    if(DriverStation.isDisabled()){
    MOTOR_P = SmartDashboard.getNumber("Motor P Value", 1/2900);
    MOTOR_I = SmartDashboard.getNumber("Motor I Value", 0);
    MOTOR_D = SmartDashboard.getNumber("Motor D Value", 0);
    tuningConstants.setPID(MOTOR_P, MOTOR_I, MOTOR_D);
    intakeMotor.withGains(tuningConstants);
    }
   }
//    public void reverseIntake(){
//     intakeMotorSide.setVoltage(-3.5);
//     intakeMotorBottom.setVoltage(-3.5);
//    }
   
   public void stopIntake() {
    intakeMotor.setVelocity(0);
   }

//    public void deployIntake() {
//     if (intakeMotorPivot.getEncoder().getPosition() > -35) {
//         intakeMotorPivot.setVoltage(-5);
//         //stopIntake();
//         Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", true);
//     }
//         else {
//             intakeMotorPivot.setVoltage(0);
//             //runIntake();
//             Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", false);
//         }
//     }

//     public void stowIntake() {
//         if (intakeMotorPivot.getEncoder().getPosition() < -10) {
//             intakeMotorPivot.setVoltage(5);
//             //runIntake();
//             Logger.recordOutput(INTAKE.LOG_PATH + "Stow ", true);
//     }
//         else {
//             //stopIntake();
//             intakeMotorPivot.setVoltage(0);
//             Logger.recordOutput(INTAKE.LOG_PATH + "Stow ", false);
//         }
//     }

//     public void stopIntakePivot() {
//         intakeMotorPivot.setVoltage(0);
//    }

//    public Command runIntakeCommand(){
//     return this.runOnce(() -> runIntake());
//    }

//    public Command reverseIntakeCommand(){
//     return this.runOnce(() -> reverseIntake());
//    }

   public Command stopIntakeCommand(){
    return this.runOnce(() -> stopIntake());
   }

//    public Command deployIntakeCommand(){
//     return this.run(() -> deployIntake());
//    }

//    public Command stowIntakeCommand(){
//     return this.run(() -> stowIntake());
//    }

//    public Command stopIntakePivotCommand(){
//     return this.runOnce(() -> stopIntakePivot());
//    }
 
   @Override
   public void periodic(){
    updatePID();
   }

}
