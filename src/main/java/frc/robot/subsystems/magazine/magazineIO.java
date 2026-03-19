package frc.robot.subsystems.magazine;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;
import frc.robot.util.tunable.FlywheelConstants;

/**
 * indexer interface
 */
@GenerateEmptyIO
public interface MagazineIO {
    /**
     * indexer inputs class
     */
    @AutoLog
    public class MagazineInputs {
        public AngularVelocity magazineVelocity = RotationsPerSecond.of(0);
        public Voltage magazineVoltage = Volts.of(0.0);
    }

    public void updateInputs(MagazineInputs inputs);

    public void setMagazineVoltage(double votlage);

    public void setConstants(FlywheelConstants constants);
}
