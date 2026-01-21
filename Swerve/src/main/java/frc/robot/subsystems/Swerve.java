package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Swerve extends SubsystemBase {

    private static ChassisSpeeds speedZero = new ChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1);

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();

    @Override
    public void periodic() {
        // TODO: Periodic logging
        // Logger.recordOutput(SWERVE.LOG_PATH+"Current Pose", getCurrentOdometryPosition());
        swerve.periodic();
    }

    // public void stop(){
    //     swerve.applyRequest()
    // }

    /**
     * Process raw joystick values
     * @param rawX raw x value
     * @param rawY raw y value
     * @param rawRot raw rotational value
     */
    public void drive(double rawX, double rawY, double rawRot){
        swerve.setControl(
            fieldCentric.withVelocityX(-rawX * maxSpeed)
            .withVelocityY(-rawY * maxSpeed)
            .withRotationalRate(-rawRot * maxAngularRate)
        );
    }

    // public ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
    //      ChassisSpeeds speeds = new ChassisSpeeds(-rawX, -rawY, -rawRot);
    //      return speeds;
    // }


}
