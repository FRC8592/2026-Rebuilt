package frc.robot.helpers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXControl {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    public TalonFXControl(int canId, boolean coastMode){
        motor = new TalonFX(canId);
        motorConfig = new TalonFXConfiguration();

        if (coastMode){
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }
        else{
            motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }

        motor.getConfigurator().apply(motorConfig);

        motor.set(0);

    }

    public void setVelocity(){}
    public void setPosition(){}
    public void setVoltage(){}
    public void setPercentOutput(){}

}
