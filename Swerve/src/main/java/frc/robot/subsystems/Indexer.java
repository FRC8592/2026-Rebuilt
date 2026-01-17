package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkMaxMotor;

public class Indexer extends SubsystemBase {
    DigitalInput[] sensors = new DigitalInput[3];
    NewtonMotor[] motors = new NewtonMotor[4];
    private boolean reverseIntake = false;

    public Indexer() {
        motors[0] = new SparkMaxMotor(CAN.INDEXER_MOTOR1_CAN_ID, true);
        motors[1] = new SparkMaxMotor(CAN.INDEXER_MOTOR2_CAN_ID, true);
        motors[2] = new SparkMaxMotor(CAN.INDEXER_MOTOR3_CAN_ID, true);
        motors[3] = new SparkMaxMotor(CAN.INDEXER_MOTOR4_CAN_ID, true);

        motors[0].setIdleMode(IdleMode.kBrake);

        motors[1].setIdleMode(IdleMode.kBrake);
        // motors[1].setFollowerTo(motors[0], false); //doesn't work

        motors[2].setIdleMode(IdleMode.kBrake);
        motors[3].setIdleMode(IdleMode.kBrake);

        sensors[0] = new DigitalInput(INDEXER.INDEXER_BEAM_BREAK_1_PORT);
        sensors[1] = new DigitalInput(INDEXER.INDEXER_BEAM_BREAK_2_PORT);
        sensors[2] = new DigitalInput(INDEXER.INDEXER_BEAM_BREAK_3_PORT);

    }

    /**
     * Checks to see if the beambreak detects a football at the given storage point
     * 
     * @param storagePoint given launcher is the front, the front storage point is
     *                     1, the middle is 2, and the back is 3
     * @return whewther a football is detected at given storage point as a boolean
     */
    public boolean hasBall(int storagePoint) {
        boolean bool = !sensors[storagePoint - 1].get();
        Logger.recordOutput(INDEXER.LOG_PATH + "Sensor " + storagePoint, bool);

        if (bool) {
            return true;
        }
        return false;
    }

    /**
     * Checks to see if there is a football anywhere within the indexer
     * 
     * @return if there is no football detected anywhere in the indexer
     */
    public boolean hasBall() {
        for (int i = 1; i <= sensors.length; i++) {
            if (hasBall(i)) {
                return true;
            }

        }
        return false;

    }

    /**
     * Finds the number of balls in the indexer
     * 
     * @return number of balls in the indexer
     */
    public int getBallCount() {
        int count = 0;
        for (int i = 1; i <= sensors.length; i++) {
            if (hasBall(i))
                count++;
        }

        return count;
    }

    /**
     * Runs given motor at given percentage
     * 
     * @param motorPos position of motor starting from 0 - 3 (intake -> launcher)
     * @param percent  scaled from -1 to 1
     */
    public void run(int motorPos, double percent) {
        motors[motorPos].setPercentOutput(percent);

    }

    public void runBeforeShoot(double percent) {
        motors[0].setPercentOutput(percent);
        motors[1].setPercentOutput(percent);
        motors[2].setPercentOutput(percent);
    }

    public void shoot(double percent){
        motors[3].setPercentOutput(percent);
        motors[2].setPercentOutput(percent);
    }

    public void stopShoot(){
        motors[3].setPercentOutput(0);
        motors[2].setPercentOutput(0);
    }

    /**
     * Runs all motors at given percentage
     * 
     * @param percent scaled from -1 to 1
     */
    public void runAll(double percent) {
        for (NewtonMotor m : motors)
            m.setPercentOutput(percent);
    }

    /**
     * Stops specified motor
     * 
     * @param motorPos position of motor starting from 0 - 3 (intake -> launcher)
     */
    public void stop(int motorPos) {
        motors[motorPos].setPercentOutput(0);
    }

    // Stops all motors from running
    public void stopAll() {
        for (NewtonMotor m : motors)
            m.setPercentOutput(0);
    }

    /**
     * Runs all motors at specified percentage
     * 
     * @param percent scaled -1 to 1
     * @return command to run motors at specified percentage
     */
    public Command setMotorPercentOutputCommand(double percent) {
        return this.run(() -> runAll(percent)).finallyDo(() -> stopAll());

    }

    /**
     * Runs specified motor at given percentage
     * 
     * @param motorPos position of motor starting from 0 - 3 (intake -> launcher)
     * @param percent  scaled from -1 to 1
     * @return Command to run given motor at given percentage
     */
    public Command setMotorPercentOutputCommand(int motorPos, double percent) {
        return this.run(() -> run(motorPos, percent)).finallyDo(() -> stop(motorPos));
    }

    /**
     * Stops all motors from running
     * 
     * @return the command to stop the motor
     */
    public Command stopMotorCommand() {
        return this.runOnce(() -> stopAll());
    }

    /**
     * Stops specified motor from running
     * 
     * @param motorPos position of motor starting from 0 - 3 (intake -> launcher)
     * @return Command to stop specified motor
     */
    public Command stopMotorCommand(int motorPos) {
        return this.runOnce(() -> stop(motorPos));
    }

    public void autoIndex() {
        boolean s1 = hasBall(1); // shooter
        boolean s2 = hasBall(2); // middle
        boolean s3 = hasBall(3);

        stop(3);

        if (reverseIntake) {
            run(0, -1);
            run(1, -1);
            run(2, -1);
        } else if (!s1) {
            runBeforeShoot(1);

        } else if (!s2) {
            stop(2);
            run(1, 1);
            run(0, 1);

        } else {
            stopAll();
        }
    }

    public void setReverse(boolean reverse) {
        reverseIntake = reverse;
    }

    public Command setIndexerReverseCommand() {
        return this.runOnce(() -> this.setReverse(true));
    }

    public Command setIndexerNormalCommand() {
        return this.runOnce(() -> this.setReverse(false));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(INDEXER.LOG_PATH + "ballCount", getBallCount());
        Logger.recordOutput(INDEXER.LOG_PATH + "indexerHasBall", hasBall());
        Logger.recordOutput(INDEXER.LOG_PATH + "3 balls", getBallCount() == 3);

        // Logger.recordOutput(INDEXER.LOG_PATH + "indexerFrontHasBall", hasBall(1));
        // Logger.recordOutput(INDEXER.LOG_PATH + "indexerMiddleHasBall", hasBall(2));
        // Logger.recordOutput(INDEXER.LOG_PATH + "indexerBackHasBall", hasBall(3));
    }

}