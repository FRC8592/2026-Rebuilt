package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;
// import frc.robot.helpers.motor.spark.*;
import frc.robot.helpers.motor.talonfx.Falcon500Motor;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;   
import frc.robot.Constants.*;
// import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.ThisIsSparkFlexMotor;
// import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;

public class Intake extends SubsystemBase{
    // Drive intake wheels
    private NewtonMotor intakeMotor;
    // Lift to deploy and stow intake

   public Intake() {
   
    // Create the intake wheel and lift motors
    intakeMotor = new SparkFlexMotor(CAN.INTAKE_MOTOR_CAN_ID, false);


   }

   public void setPercentOut(NewtonMotor intakeMotor, double percent){
        intakeMotor.setPercentOutput(percent);

   }

   public void stop(NewtonMotor intakeMotor){
        intakeMotor.setPercentOutput(0);
   }

    public Command setIntakeCommand (double percent){
    return this.run(() -> setPercentOut(intakeMotor, percent));
    }

    public Command setOuttakeCommand (double percent){
    return this.run (() -> setPercentOut(intakeMotor, percent));
    }

   public Command stopIntakeCommand(){
    return this.runOnce(() -> stop(intakeMotor));
   }

   public double rotationstoDegrees(double motorRotations){
    return motorRotations * 1/(INTAKE.INTAKE_DEGREES_TO_MOTOR_ROTATIONS);
   }

   public double degreestoRotations(double degrees){
    return degrees * INTAKE.INTAKE_DEGREES_TO_MOTOR_ROTATIONS;
   }
 

}
