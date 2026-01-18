package frc.robot.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkFlex.*;
// import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;   
// import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.ThisIsSparkFlexMotor;
// import frc.robot.helpers.PIDProfile;
// import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.Constants.CAN;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.Falcon500Motor;

public class Intake extends SubsystemBase{
    // Drive intake wheels
   private NewtonMotor intakeMotor; 
    // Lift to deploy and stow intake
    //private SparkFlex intakeMotorPivot;

    // private SparkClosedLoopController closedLoopController;

   public Intake() {
   
        // Create the intake wheel and lift motors
        intakeMotor = new Falcon500Motor(CAN.INTAKE_MOTOR_CAN_ID, true);
        //intakeMotorPivot = new SparkFlex(CAN.PIVOT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

   }

   public void runIntake() {
    intakeMotor.setVoltage(4);
   }

   public void reverseIntake(){
    intakeMotor.setVoltage(-3.5);
   }
   
   public void stopIntake() {
    intakeMotor.setVoltage(0);
   }

//    public void deployIntake() {
    // if (intakeMotorPivot.getEncoder().getPosition() > -35) {
    //     intakeMotorPivot.setVoltage(-5);
    //     //stopIntake();
    //     Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", true);
    // }
    //     else {
    //         //intakeMotorPivot.setVoltage(0);
    //         //runIntake();
    //         Logger.recordOutput(INTAKE.LOG_PATH + "Deploy ", false);
    //     }
    // }

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

   public Command runIntakeCommand(){
    return this.runOnce(() -> runIntake());
   }

   public Command reverseIntakeCommand(){
    return this.runOnce(() -> reverseIntake());
   }

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
 
//    @Override
//    public void periodic(){
//     Logger.recordOutput(INTAKE.LOG_PATH + "Pivot Motor Rotations", intakeMotorPivot.getEncoder().getPosition());
//    }

}
