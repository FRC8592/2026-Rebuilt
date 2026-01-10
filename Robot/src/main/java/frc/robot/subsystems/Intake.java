package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

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
    private SparkFlex intakeMotorSide;
    private SparkFlex intakeMotorBottom;
    // Lift to deploy and stow intake
    private SparkFlex intakeMotorPivot;

    private SparkClosedLoopController closedLoopController;

   public Intake() {
   
        // Create the intake wheel and lift motors
        intakeMotorSide = new SparkFlex(CAN.INTAKE_MOTOR_SIDE_CAN_ID,MotorType.kBrushless);
        intakeMotorBottom = new SparkFlex(CAN.INTAKE_MOTOR_BOTTOM_CAN_ID,MotorType.kBrushless);
        intakeMotorPivot = new SparkFlex(CAN.PIVOT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

   }

   public void runIntake() {
    intakeMotorSide.setVoltage(4);
    intakeMotorBottom.setVoltage(4);
   }

   public void reverseIntake(){
    intakeMotorSide.setVoltage(-3.5);
    intakeMotorBottom.setVoltage(-3.5);
   }
   
   public void stopIntake() {
    intakeMotorSide.setVoltage(0);
    intakeMotorBottom.setVoltage(0);
   }

   public void deployIntake() {
    if (intakeMotorPivot.getEncoder().getPosition() > -35) {
        intakeMotorPivot.setVoltage(-5);
        //stopIntake();
        Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", true);
    }
        else {
            intakeMotorPivot.setVoltage(0);
            //runIntake();
            Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", false);
        }
    }

    public void stowIntake() {
        if (intakeMotorPivot.getEncoder().getPosition() < -10) {
            intakeMotorPivot.setVoltage(5);
            //runIntake();
            Logger.recordOutput(INTAKE.LOG_PATH + "Stow ", true);
    }
        else {
            //stopIntake();
            intakeMotorPivot.setVoltage(0);
            Logger.recordOutput(INTAKE.LOG_PATH + "Stow ", false);
        }
    }

    public void stopIntakePivot() {
        intakeMotorPivot.setVoltage(0);
   }

   public Command runIntakeCommand(){
    return this.runOnce(() -> runIntake());
   }

   public Command reverseIntakeCommand(){
    return this.runOnce(() -> reverseIntake());
   }

   public Command stopIntakeCommand(){
    return this.runOnce(() -> stopIntake());
   }

   public Command deployIntakeCommand(){
    return this.run(() -> deployIntake());
   }

   public Command stowIntakeCommand(){
    return this.run(() -> stowIntake());
   }

   public Command stopIntakePivotCommand(){
    return this.runOnce(() -> stopIntakePivot());
   }
 
   @Override
   public void periodic(){
    Logger.recordOutput(INTAKE.LOG_PATH + "Pivot Motor Rotations", intakeMotorPivot.getEncoder().getPosition());
   }

}
