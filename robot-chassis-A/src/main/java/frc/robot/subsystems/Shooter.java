package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER;


public class Shooter extends SubsystemBase{
    private TalonFX flywheelMotor;  //big one
    private TalonFX backwheelMotor; //smaller wheels
    private TalonFXConfiguration flywheelConfiguration;
    private TalonFXConfiguration backwheelConfiguration;

    private VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private VelocityVoltage backwheelVelocityRequest = new VelocityVoltage(0);

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

        backwheelConfiguration.Slot0.kP = SHOOTER.BACKWHEEL_P;
        backwheelConfiguration.Slot0.kI = SHOOTER.BACKWHEEL_I;
        backwheelConfiguration.Slot0.kD = SHOOTER.BACKWHEEL_D;
        backwheelConfiguration.Slot0.kV = SHOOTER.BACKWHEEL_V;

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

        SmartDashboard.putNumber("Vi", SHOOTER.FLYWHEEL_VI);
    }


    /**
     * Run the shooter motor at a set speed in RPM.
     * The left shooter motor is the only motor in use on the Shooter.
     * Utilizing both motors proved to be too powerful for the shooter.
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    public void runAtSpeed(double desiredRPM){
        double flyWheelMotorVelocity = SmartDashboard.getNumber("Vi", SHOOTER.FLYWHEEL_VI);
        double backwheelMotorVelocity = flyWheelMotorVelocity * WHEEL_RATIO;

        flywheelMotor.setControl(flywheelVelocityRequest.withSlot(0).withVelocity(flyWheelMotorVelocity));
        backwheelMotor.setControl(backwheelVelocityRequest.withSlot(0).withVelocity(backwheelMotorVelocity));
    }


    /**
     * Command to run the shooter motor at a set speed.
     * @return Returns a command for running the RunAtSpeed method once.
     */
    public Command runAtSpeedCommand(){
        return this.runOnce(() -> runAtSpeed(SHOOTER.FLYWHEEL_VI));
    }


    /**
     * Update the PID values on the fly on the shooter motor.
     * The NEO Motors do not allowed their PID Profile to be updated while running.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID(){
        flywheelConfiguration.Slot0.kP = SmartDashboard.getNumber("fP", SHOOTER.FLYWHEEL_P); 
        flywheelConfiguration.Slot0.kI = SmartDashboard.getNumber("fI", SHOOTER.FLYWHEEL_I); 
        flywheelConfiguration.Slot0.kD = SmartDashboard.getNumber("fD", SHOOTER.FLYWHEEL_D); 
        flywheelConfiguration.Slot0.kV = SmartDashboard.getNumber("fV", SHOOTER.FLYWHEEL_V); 

        backwheelConfiguration.Slot0.kP = SmartDashboard.getNumber("bP", SHOOTER.BACKWHEEL_P); 
        backwheelConfiguration.Slot0.kI = SmartDashboard.getNumber("bI", SHOOTER.BACKWHEEL_I); 
        backwheelConfiguration.Slot0.kD = SmartDashboard.getNumber("bD", SHOOTER.BACKWHEEL_D); 
        backwheelConfiguration.Slot0.kV = SmartDashboard.getNumber("bV", SHOOTER.BACKWHEEL_V); 

        flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        backwheelMotor.getConfigurator().apply(backwheelConfiguration);
    }


    /**
     * Stops the shooter motor, thus bringing the flywheel to a gradual stop.
     * 
     * Utilized setVoltage instead of Velocity Control to prevent power being used to stop flywheel.
     */
    public void stopShooter(){
        flywheelMotor.setVoltage(0.0);
        backwheelMotor.setVoltage(0.0);
    }


    /**
     * Command form of the stopShooter method.
     * @return Returns a command to run the stopShooter method once.
     */
    public Command stopShooterCommand(){
        return this.runOnce(() -> stopShooter());
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
     * Simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub.
     * Very much a theoretical implementation, simply putting in just the math.
     * @param theta The angle of the shooter.
     * @param dis The distance from the shooter to the center of the hub.
     * @return Returns velocity ball needs to achieve.
     */
    // public double DistanceToRPM(double theta, double dis){
    //     return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(SHOOTER.SHOOTER_HEIGHT + Math.tan(theta) - SHOOTER.HUB_HEIGHT)));
    // }


    /**
     * Periodic method, primarily for logging.
     */
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Flywheel Velocity RPM", getVelocityFlywheel() * 60);
        SmartDashboard.putNumber("Backwheel Velocity RPM", getVelocityBackwheel() * 60);
    }
        
}