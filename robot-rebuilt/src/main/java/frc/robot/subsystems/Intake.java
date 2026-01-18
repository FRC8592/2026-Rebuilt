package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
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
    intakeMotor = new Falcon500Motor(CAN.INTAKE_MOTOR_CAN_ID, true);

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

}
