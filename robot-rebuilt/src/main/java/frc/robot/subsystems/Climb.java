
package frc.robot.subsystems;

// import org.littletonrobotics.junction.Logger;

//import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;
// import frc.robot.helpers.motor.spark.*;
//import frc.robot.helpers.motor.talonfx.Falcon500Motor;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;   
import frc.robot.Constants.*;
// import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.ThisIsSparkFlexMotor;
// import frc.robot.helpers.PIDProfile;
// import frc.robot.helpers.motor.NewtonMotor;

public class Climb extends SubsystemBase{
    // Drive intake wheels
    private NewtonMotor climbMotor;
    // Lift to deploy and stow intake

   public Climb() {
   
    // Create the intake wheel and lift motors
    climbMotor = new SparkFlexMotor(CLIMB.CLIMB_CAN_ID, false);


   }

   public void setPercentOut(NewtonMotor climbMotor, double percent){
        climbMotor.setPercentOutput(percent);

   }

   public void stop(NewtonMotor climbMotor){
        climbMotor.setPercentOutput(0);
   }

    public Command setClimbCommand (double percent){
    return this.run(() -> setPercentOut(climbMotor, percent));
    }

    public Command setReverseCommand (double percent){
    return this.run (() -> setPercentOut(climbMotor, -percent));
    }

   public Command stopCommand(){
    return this.runOnce(() -> stop(climbMotor));
   }
 

}
