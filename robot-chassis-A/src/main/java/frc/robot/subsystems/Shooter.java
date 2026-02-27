package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER;
import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase{
    private TalonFX flywheelMotor;  //big one
    private TalonFX backwheelMotor; //smaller wheels
    private TalonFXConfiguration flywheelConfiguration;
    private TalonFXConfiguration backwheelConfiguration;

    private VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private VelocityVoltage backwheelVelocityRequest = new VelocityVoltage(0);

    private double PF_OLD;
    private double IF_OLD;
    private double DF_OLD;
    private double VF_OLD;

    private double PB_OLD;
    private double IB_OLD;
    private double DB_OLD;
    private double VB_OLD;

    private final double WHEEL_RATIO = SHOOTER.FLYWHEEL_DIAMETER_INCHES/SHOOTER.BACKWHEEL_DIAMETER_INCHES;

    /**
     * Constructor for the Shooter subsystem
     * 
     * Instantiate the motor with Initial PID values from the CONSTANTS Class
     * 
     * Display PID Values on SmartDashboard
     * 
     * Set the current limit of the shooter motor
     * 
     */
    public Shooter(){

        flywheelMotor = new TalonFX(SHOOTER.FLYWHEEL_MOTOR_CAN_ID);
        backwheelMotor = new TalonFX(SHOOTER.BACKWHEEL_MOTOR_CAN_ID);
        flywheelConfiguration = new TalonFXConfiguration();
        backwheelConfiguration = new TalonFXConfiguration();


        flywheelConfiguration.Slot0.kP = SHOOTER.FLYWHEEL_P; 
        flywheelConfiguration.Slot0.kI = SHOOTER.FLYWHEEL_I;
        flywheelConfiguration.Slot0.kD = SHOOTER.FLYWHEEL_D;
        flywheelConfiguration.Slot0.kV = SHOOTER.FLYWHEEL_V; 

        flywheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        backwheelConfiguration.Slot0.kP = SHOOTER.BACKWHEEL_P;
        backwheelConfiguration.Slot0.kI = SHOOTER.BACKWHEEL_I;
        backwheelConfiguration.Slot0.kD = SHOOTER.BACKWHEEL_D;
        backwheelConfiguration.Slot0.kV = SHOOTER.BACKWHEEL_V;

        backwheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        backwheelMotor.getConfigurator().apply(backwheelConfiguration);

        // TODO: For tuning, PID and velocity values are displayed on SmartDashboard, remove before competition
        SmartDashboard.putNumber("fP", SHOOTER.FLYWHEEL_P);
        SmartDashboard.putNumber("fI", SHOOTER.FLYWHEEL_I);
        SmartDashboard.putNumber("fD", SHOOTER.FLYWHEEL_D);
        SmartDashboard.putNumber("fV", SHOOTER.FLYWHEEL_V);

        SmartDashboard.putNumber("bP", SHOOTER.BACKWHEEL_P);
        SmartDashboard.putNumber("bI", SHOOTER.BACKWHEEL_I);
        SmartDashboard.putNumber("bD", SHOOTER.BACKWHEEL_D);
        SmartDashboard.putNumber("bV", SHOOTER.BACKWHEEL_V);

        SmartDashboard.putNumber("Vi_Shooter", SHOOTER.FLYWHEEL_VI);
    }


    /**
     * Run the shooter motor at a set speed in RPM.
     * The left shooter motor is the only motor in use on the Shooter.
     * Utilizing both motors proved to be too powerful for the shooter.
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    public void runAtSpeed(double desiredRPM){
        double flyWheelMotorVelocity = SmartDashboard.getNumber("Vi_Shooter", SHOOTER.FLYWHEEL_VI) / 60; // Convert from RPM to RPS for the motor controller
        double backwheelMotorVelocity = flyWheelMotorVelocity * WHEEL_RATIO;

        // To run at raw power
        // flywheelMotor.setVoltage(12);
        // backwheelMotor.setVoltage(12);
        flywheelMotor.setControl(flywheelVelocityRequest.withSlot(0).withVelocity(flyWheelMotorVelocity));
        backwheelMotor.setControl(backwheelVelocityRequest.withSlot(0).withVelocity(backwheelMotorVelocity));
    }


    /**
     * Command to run the shooter motor at a set speed.
     * @return Returns a command for running the RunAtSpeed method once.
     */
    public Command runAtSpeedCommand(double desiredRPM){
        return this.runOnce(() -> runAtSpeed(desiredRPM));
    }


    /**
     * Update the PID values on the fly on the shooter motor.
     * The NEO Motors do not allowed their PID Profile to be updated while running.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID(){

        double PF = SmartDashboard.getNumber("fP", SHOOTER.FLYWHEEL_P); 
        double IF = SmartDashboard.getNumber("fI", SHOOTER.FLYWHEEL_I); 
        double DF = SmartDashboard.getNumber("fD", SHOOTER.FLYWHEEL_D); 
        double VF = SmartDashboard.getNumber("fV", SHOOTER.FLYWHEEL_V); 

        double PB = SmartDashboard.getNumber("bP", SHOOTER.BACKWHEEL_P); 
        double IB = SmartDashboard.getNumber("bI", SHOOTER.BACKWHEEL_I); 
        double DB = SmartDashboard.getNumber("bD", SHOOTER.BACKWHEEL_D); 
        double VB = SmartDashboard.getNumber("bV", SHOOTER.BACKWHEEL_V); 

         if(PF != PF_OLD || IF != IF_OLD || DF != DF_OLD || PB != PB_OLD || IB != IB_OLD || DB != DB_OLD){
            flywheelConfiguration.Slot0.kP = PF; 
            flywheelConfiguration.Slot0.kI = IF;
            flywheelConfiguration.Slot0.kD = DF;
            flywheelConfiguration.Slot0.kV = VF; 

            backwheelConfiguration.Slot0.kP = PB; 
            backwheelConfiguration.Slot0.kI = IB;
            backwheelConfiguration.Slot0.kD = DB;
            backwheelConfiguration.Slot0.kV = VB; 

            PF_OLD = PF;
            IF_OLD= IF;
            DF_OLD = DF;
            VF_OLD = VF;

            PB_OLD = PB;
            IB_OLD= IB;
            DB_OLD = DB;
            VB_OLD = VB;

            flywheelMotor.getConfigurator().apply(flywheelConfiguration);
            backwheelMotor.getConfigurator().apply(backwheelConfiguration);
        }
    }


    /**
     * Stops the shooter motor, thus bringing the flywheel to a gradual stop.
     * 
     * Utilized setVoltage instead of Velocity Control to prevent power being used to stop flywheel.
     */
    public void stop(){
        flywheelMotor.setVoltage(0.0);
        backwheelMotor.setVoltage(0.0);
    }


    /**
     * Command form of the stopShooter method.
     * @return Returns a command to run the stopShooter method once.
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }


    /**
     * Purpose is to see if the motor is achieving the
     * @return Returns velocity of the flywheel motor in RPS.
     */
    public double getVelocityFlywheel(){
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Purpose is to see if the motor is achieving the
     * @return Returns velocity of the backwheel motor in RPS.
     */
    public double getVelocityBackwheel(){
        return backwheelMotor.getVelocity().getValueAsDouble();
    }


    /**
     * Periodic method, primarily for logging.
     */
    @Override
    public void periodic(){
        Logger.recordOutput("Flywheel Velocity RPM", getVelocityFlywheel() * 60);
        Logger.recordOutput("Backwheel Velocity RPM", getVelocityBackwheel() * 60 / (WHEEL_RATIO * 1.0));
    }
        
}