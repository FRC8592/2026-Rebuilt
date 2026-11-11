package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;
import java.lang.Math;
import static edu.wpi.first.units.Units.*;
import frc.robot.helpers.TalonFXControl;

public class Turret extends SubsystemBase {
    private TalonFXControl turretMotor;

    private final DutyCycleEncoder E1;
    private final DutyCycleEncoder E2;

    private double rawTargetAngle;
    private double targetAngle;
    private double turretAngleOffset = 0;

    public Turret() {
        E1 = new DutyCycleEncoder(0, 360, TURRET.E1_OFFSET.in(Degrees));
        E2 = new DutyCycleEncoder(1, 360, TURRET.E2_OFFSET.in(Degrees));

        turretMotor = new TalonFXControl(TURRET.TURRET_MOTOR_CAN_ID, false);
        turretMotor.createSlot0();

        turretMotor.overrideBrakeDurNeutral(true);
        turretMotor.inverted(InvertedValue.Clockwise_Positive);
        turretMotor.statorCurrentLimit(TURRET.CURRENT_LIMIT);

        turretMotor.setForwardSoftLimit(TURRET.FORWARD_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS);
        turretMotor.setReverseSoftLimit(TURRET.REVERSE_LIMIT * TURRET.DEGREES_TO_MOTOR_ROTATIONS);

        turretMotor.setPID(TURRET.TURRET_P.in(Volts), TURRET.TURRET_I.in(Volts), TURRET.TURRET_D.in(Volts),
                TURRET.TURRET_S.in(Volts), TURRET.TURRET_V.in(Volts), TURRET.TURRET_A.in(Volts), 0);
        turretMotor.applyConfig();

        SmartDashboard.putNumber("tP", TURRET.TURRET_P.in(Volts));
        SmartDashboard.putNumber("tI", TURRET.TURRET_I.in(Volts));
        SmartDashboard.putNumber("tD", TURRET.TURRET_D.in(Volts));

        SmartDashboard.putNumber("Turret Voltage", 0);
        SmartDashboard.putNumber("turretAngleOffset", turretAngleOffset);

        Logger.recordOutput("turretAngleOffset", turretAngleOffset);

        this.resetPos();
    }

    public void TurrettoAngle(Pose2d robotPosition, double angle) {
        double robotAngle = robotPosition.getRotation().getDegrees();
        double target = angle - robotAngle - TURRET.TURRET_ANGLE_OFFSET;
        rawTargetAngle = target;
        if (Math.abs(target) > TURRET.MAX_ROTATION_LIMIT)
            target -= Math.signum(target) * 360;
        logAngle(target);
        turretMotor.setPositionControl(target * TURRET.DEGREES_TO_MOTOR_ROTATIONS, 0);
        turretMotor.setVelocityControl(target * TURRET.DEGREES_TO_MOTOR_ROTATIONS, 0);
    }

    public void basicTurretToPos(double angle) {
        double voltage = SmartDashboard.getNumber("Turret Voltage", 0);
        turretMotor.setVoltage(voltage);
    }

    public Command basicTurretToPosCommand(double angle) {
        return this.runOnce(() -> basicTurretToPos(angle));
    }

    // Not normally used; we want the turret to hold position with the motor
    public void stop() {
        turretMotor.stop();
    }

    // Not normally used; we want the turret to hold position with the motor
    public Command stopTurretCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Get the encoder values the define the turret zero position.
     */
    public void resetPos() {
        turretMotor.setPositionControl(0, 0);
    }

    public double getRawTurretAngle() {
        return rawTargetAngle;
    }

    public boolean canShoot() {
        return Math.abs(getAngle() - getTargetAngle()) <= TURRET.TURRET_TOLERANCE;
    }

    public void turretRight() {
        turretAngleOffset -= 5;
        SmartDashboard.putNumber("turretAngleOffset", turretAngleOffset);
        Logger.recordOutput("turretAngleOffset", turretAngleOffset);
    }

    public Command turretRightCommand() {
        return this.runOnce(() -> turretRight());
    }

    public void turretLeft() {
        turretAngleOffset += 5;
        SmartDashboard.putNumber("turretAngleOffset", turretAngleOffset);
        Logger.recordOutput("turretAngleOffset", turretAngleOffset);
    }

    public Command turretLeftCommand() {
        return this.runOnce(() -> turretLeft());
    }

    public double getTurretAngleOffset() {
        return turretAngleOffset;
    }

    /**
     * Command to reset the zero position of the turret. This is used to recalibrate the turret, not for live control
     */
    public Command resetPosCommand() {
        return this.runOnce(() -> resetPos());
    }

    public void holdPosition() {
        double holdPos = turretMotor.getPosition();
        turretMotor.setPositionControl(holdPos, 0);
    }

    public double getAngle() {
        return turretMotor.getPosition() / (TURRET.DEGREES_TO_MOTOR_ROTATIONS);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void logAngle(double angle) {
        targetAngle = angle;
    }

    @Override
    public void periodic() {

        Logger.recordOutput("E1", E1.get());
        Logger.recordOutput("E2", E2.get());

        Logger.recordOutput(TURRET.LOG_PATH + "Motor Angle",
                turretMotor.getPosition()
                        * (1 / TURRET.DEGREES_TO_MOTOR_ROTATIONS));
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Rotations",
                turretMotor.getPosition()); // rotations per second
        Logger.recordOutput(TURRET.LOG_PATH + "Motor Voltage",
                turretMotor.getVoltage());

        Logger.recordOutput(TURRET.LOG_PATH + "Turret Target Angle", targetAngle);

    }
}
