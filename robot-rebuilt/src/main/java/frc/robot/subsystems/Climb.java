
package frc.robot.subsystems;

//import com.ctre.phoenix6.CANBus;
//import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.helpers.motor.talonfx.TalonFXMotor;
import frc.robot.helpers.motor.MotorConstants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;   
import frc.robot.Constants.*;
// import frc.robot.helpers.motor.NewtonMotor;
// import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.ThisIsSparkFlexMotor;
// import frc.robot.helpers.PIDProfile;
// import frc.robot.helpers.motor.NewtonMotor;

public class Climb extends SubsystemBase{ 
    // Drive intake wheels
    //private TalonFXMotor climbMotor;
    // Lift to deploy and stow intake

    MotorConstants motorConstants = null;

    TalonFX climbMotor = new TalonFX(3);
   public Climb() {

     var climbConfiguration = new TalonFXConfiguration();
     climbConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     climbMotor.getConfigurator().apply(climbConfiguration);

    // Create the intake wheel and lift motors
    //climbMotor = new TalonFXMotor(CLIMB.CLIMB_CAN_ID, motorConstants);

   }

   public void setPercentOut(TalonFX climbMotor, double percent){
        climbMotor.setVoltage(percent);

   }

   public void stop(TalonFX climbMotor){
        climbMotor.setVoltage(0);
   }

    public Command setClimbCommand (double percent){
    return this.runOnce(() -> setPercentOut(climbMotor, 4));
    }

    public Command setReverseCommand (double percent){
    return this.runOnce (() -> setPercentOut(climbMotor, -2));
    }

   public Command stopCommand(){
    return this.runOnce(() -> stop(climbMotor));
   }
 

}
