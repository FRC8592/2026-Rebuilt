// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.*;

public class ExampleSubsystem extends SubsystemBase {
  private Intake testingIntake;
  private boolean IndexerIntake;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    testingIntake = new Intake();
    IndexerIntake = false;
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void TestingMethod(){

  }
 public Command EjectLunite(){
  return new DeferredCommand(() -> EjectLuniteIntake(), Set.of(testingIntake))
  .andThen(DefaultRunIntake())
  .andThen(StowIntake());
 }

 public Command IntakeLunite(){
  //This needs to be changed, methodology behind it is to rotate the indexer down, and run the intake until Indexer knows it has the ball
  return new DeferredCommand(() -> DeployIntake(), Set.of(testingIntake))
  .andThen(testingIntake.setIntakeSideCommand(0.7));
 }

 public Command StowIntake(){
  return this.runOnce(() -> testingIntake.setToPositionCommand(INTAKE.STOW_PIVOT_INTAKE));
 }

 public Command DeployIntake(){
  return this.runOnce(() -> testingIntake.setToPositionCommand(INTAKE.SET_PIVOT_INTAKE));
 }

 public Command EjectLuniteIntake(){
  return this.runOnce(() -> testingIntake.setToPositionCommand(INTAKE.EJECT_LUNITE_POSITION));
 }

 public Command DefaultRunIntake(){
  return this.runOnce(() -> testingIntake.runIntakeToPositionCommand());
 }

}
