package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.lang.Math;

import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;
// import frc.robot.helpers.motor.spark.SparkFlexMotor;


public class Shooter extends SubsystemBase{
    private KrakenX60Motor LeftShooterMotor;
    private KrakenX60Motor RightShooterMotor;
    private PIDProfile MotorPID;

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

        //TODO: Implement Kraken Motors
        //TODO: Possibly implement Motion Magic
        //TODO: Remove one of the shooter motors if necessary
        //RightShooterMotor = new KrakenX60Motor(SHOOTER.RIGHT_SHOOTER_MOTOR, true);
        //LeftShooterMotor = new KrakenX60Motor(SHOOTER.LEFT_SHOOTER_MOTOR, false);
        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(SHOOTER.MOTOR_P, SHOOTER.MOTOR_I, SHOOTER.MOTOR_D);
        //LeftShooterMotor.withGains(MotorPID);
        //LeftShooterMotor.setCurrentLimit(40);
        //LeftShooterMotor.configureMotionMagic(10, 5);
        //RightShooterMotor.setCurrentLimit(40);

        // TODO: For tuning, PID and velocity values are displayed on SmartDashboard, remove before competition
        SmartDashboard.putNumber("P", 0.1);
        SmartDashboard.putNumber("I", 0.0);
        SmartDashboard.putNumber("D", 0.0);
        SmartDashboard.putNumber("Vi", 500);
    }


    /**
     * Run the shooter motor at a set speed in RPM.
     * The left shooter motor is the only motor in use on the Shooter.
     * Utilizing both motors proved to be too powerful for the shooter.
     * @param desiredRPM The desired RPM we want the shooter motor to achieve.
     */
    public void runAtSpeed(double desiredRPM){
        double RPM = SmartDashboard.getNumber("Vi", 0);
        //LeftShooterMotor.setVelocity(RPM);
        //RightShooterMotor.setVelocity(RPM);
    }


    /**
     * Command to run the shooter motor at a set speed.
     * @return Returns a command for running the RunAtSpeed method once.
     */
    public Command runAtSpeedCommand(){
        double RPM = SmartDashboard.getNumber("Vi", 0);
        return this.runOnce(() -> runAtSpeed(RPM));
    }


    /**
     * Update the PID values on the fly on the shooter motor.
     * The NEO Motors do not allowed their PID Profile to be updated while running.
     * 
     * Thus, this method is called in disabledPeriodic() within Robot.java.
     */
    public void updatePID(){
        double P = SmartDashboard.getNumber("P", 0.001);
        double I = SmartDashboard.getNumber("I", 0.0);
        double D = SmartDashboard.getNumber("D", 0.0);
        MotorPID.setPID(P, I, D);
        //LeftShooterMotor.withGains(MotorPID);
        //RightShooterMotor.withGains(MotorPID);
    }


    /**
     * Stops the shooter motor, thus bringing the flywheel to a gradual stop.
     * 
     * Utilized PercentOutput instead of Velocity Control to prevent power being used to stop flywheel.
     */
    public void stopShooter(){
        //RightShooterMotor.setPercentOutput(0);
        //LeftShooterMotor.setPercentOutput(0);
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
     * @return Returns velocity of the shooter motor.
     */
    // public double getVelocity(){
    //     return RightShooterMotor.getVelocityRPM();
    // }


    /**
     * Simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub.
     * Very much a theoretical implementation, simply putting in just the math.
     * @param theta The angle of the shooter.
     * @param dis The distance from the shooter to the center of the hub.
     * @return Returns velocity ball needs to achieve.
     */
    public double DistanceToRPM(double theta, double dis){
        return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(SHOOTER.SHOOTER_HEIGHT + Math.tan(theta) - SHOOTER.HUB_HEIGHT)));
    }


    /**
     * Periodic method, primarily for logging.
     */
    // @Override
    // public void periodic(){
    //     System.out.println("Velocity: " + getVelocity() * 60);
    //     SmartDashboard.putNumber("Motor Velocity RPM", getVelocity() * 60);
    // }
        
}