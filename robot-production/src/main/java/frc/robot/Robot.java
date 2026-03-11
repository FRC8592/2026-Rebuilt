// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Optional;

import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDS;
import frc.robot.Constants.SHOOTER;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.vision.Vision;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
  //     .withTimestampReplay()
  //     .withJoystickReplay();


  public static Field2d FIELD = new Field2d();
  private static int periodicCounter = 0; 
  private static int tagCounter = 0; 
  private static int tagTarget =0; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    CanBridge.runTCP(); // Required for Grapplehook laser reflection sensor.

    // Logger configuration
    Logger.recordMetadata("Game", "REBUILT");
    Logger.recordMetadata("Year", "2026");
    Logger.recordMetadata("Team", "8592");

    if (isReal()) { // If running on a real robot
      String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());

      File sda1 = new File("/media/sda1/logs");
      if(sda1.exists()){
          String path = "/media/sda1/"+time+".wpilog";
          Logger.addDataReceiver(new WPILOGWriter(path));
      } else {
        File sdb1 = new File("/media/sdb1/logs");
        if(sdb1.exists()){
            String path = "/media/sdb2/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path));
        } else {
            System.err.println("UNABLE TO LOG TO A USB STICK!");
        }
      }

      LoggedPowerDistribution.getInstance(1, ModuleType.kRev);  // Enables power distribution logging
    }

    //SmartDashboard.putData(FIELD); // TODO:Do we need this?

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();
          
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      //m_robotContainer.scoring.intake.setBrakeMode();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void disabledPeriodic() {
    // Stop turret tracking and shooter speed control
    m_robotContainer.scoring.disableTracking();

    // Run vision routines
    Vision backvision = m_robotContainer.getBackVision();
    Vision sidevision = m_robotContainer.getSideVision(); 
    backvision.periodic();
    sidevision.periodic();

    int backvisionCounter = backvision.getTargets().size();
    int sideVisionCounter = sidevision.getTargets().size();

    Logger.recordOutput(LEDS.LOG_PATH + "periodicCounter", periodicCounter);

    // LED control
    if (periodicCounter %10 ==0){
      double average = tagCounter/10; 
      Logger.recordOutput(LEDS.LOG_PATH + "Average", average);
      m_robotContainer.leds.setHasTags((int)Math.round(average));

      tagCounter = 0;
      if (backvisionCounter == 2 || sideVisionCounter == 2){
      tagCounter += 2;
    }

    else if (backvisionCounter == 1 || sideVisionCounter ==1){
      tagCounter += 1;
    }

    else if (backvisionCounter == 0 || sideVisionCounter ==0){
      tagCounter += 0;
    }
      
    }

    else {
      tagCounter = 0;
    }

    Logger.recordOutput(LEDS.LOG_PATH + "tag counter", tagCounter);

    periodicCounter++; 

    m_robotContainer.leds.displayHasTagsLEDs();
  

    // Pass Red/Blue alliance information to the scoring subsystem so it can select the correct target
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()){
      m_robotContainer.scoring.setAlliance(alliance.get());
    }

    // Update PID values from SmartDashboard for all subsystems that use PID.  This allows for tuning while the robot is disabled.
    m_robotContainer.scoring.shooter.updatePID();
    m_robotContainer.scoring.indexer.updatePID();
    m_robotContainer.scoring.intake.updatePID();
    // m_robotContainer.scoring.turret.updatePID();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.scoring.disableTrackingCommand(); 

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
    public void autonomousExit() {}

  @Override
  public void teleopInit() {
    tagTarget = 0; 
    m_robotContainer.leds.displayCanShootLEDs();
    m_robotContainer.scoring.disableTrackingCommand(); 
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    //m_robotContainer.scoring.intake.setCoastMode();
     //   m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
    public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

   @Override
    public void testExit() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    
  }
}
