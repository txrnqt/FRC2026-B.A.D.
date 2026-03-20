package frc.robot.subsystems.magazine;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixSignals;
import frc.robot.util.tunable.FlywheelConstants;

/**
 * real implementation of indexer
 */
public class MagazineReal implements MagazineIO {
    private final TalonFX motor0 = new TalonFX(Constants.Magazine.motor0ID);
    private final TalonFX motor1 = new TalonFX(Constants.Magazine.motor1ID);

    private final StatusSignal<AngularVelocity> motor0Velocity = motor0.getVelocity();
    private final StatusSignal<Voltage> motor0Voltage = motor0.getMotorVoltage();

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private TalonFXConfiguration magzineConfig = new TalonFXConfiguration();
    private double desiredSpeed = 3.0;

    /** Real Indexer Implementation */
    public MagazineReal() {
        Logger.recordOutput("Magazine/desiredSpeed", desiredSpeed);

        PhoenixSignals.registerSignals(false, motor0Velocity, motor0Voltage);
        setConstants(Constants.Magazine.constants);
    }

    @Override
    public void updateInputs(MagazineInputs inputs) {
        BaseStatusSignal.refreshAll(motor0Velocity, motor0Voltage);

        inputs.magazineVelocity = motor0Velocity.getValue();

        inputs.magazineVoltage = motor0Voltage.getValue();
    }

    @Override
    public void setMagazineVoltage(double votlage) {
        motor0.setControl(velocityVoltage.withAcceleration(votlage * desiredSpeed));
    }

    @Override
    public void setConstants(FlywheelConstants constants) {
        motor1.setControl(new Follower(motor0.getDeviceID(), Constants.Magazine.motorAlingment));

        magzineConfig.MotorOutput.Inverted = Constants.Magazine.inverted;
        magzineConfig.MotorOutput.NeutralMode = Constants.Magazine.neutralMode;
        magzineConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.holdCurrent;
        magzineConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        magzineConfig.MotorOutput.PeakForwardDutyCycle = constants.maxDutyCycle;
        magzineConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        constants.pid.apply(magzineConfig.Slot0);

        motor0.getConfigurator().apply(magzineConfig);
        motor1.getConfigurator().apply(magzineConfig);
    }
}
