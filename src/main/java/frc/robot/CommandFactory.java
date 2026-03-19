package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/** Command Factory */
public class CommandFactory {

    /** Prepare flywheel and turret for shooting from a given robot pose. */
    public static Command preShoot(Supplier<Pose2d> robotPoseSupplier,
        Supplier<Translation2d> targetSupplier, Swerve swerve, Shooter shooter,
        DoubleSupplier adjustUp, DoubleSupplier adjustRight) {
        return Commands.run(() -> {
            final Translation2d target = targetSupplier.get();
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d turretPosition = robotPose
                .plus(new Transform2d(Constants.Vision.turretCenter.toPose2d().getTranslation(),
                    Rotation2d.kZero))
                .getTranslation();
            double adjustUpValue = Units.metersToFeet(adjustUp.getAsDouble());
            Rotation2d adjustRightValue = Rotation2d.fromDegrees(adjustRight.getAsDouble());
            double distance = target.getDistance(turretPosition) + adjustUpValue;
            var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), true);
            shooter.setVelocity(parameters.desiredSpeed());
            boolean facingHub = swerve.turnToRotation(adjustRight);
            boolean isOkay = parameters.isOkayToShoot();
            Logger.recordOutput("AutoShoot/facingHub", facingHub);
            Logger.recordOutput("AutoShoot/isOkay", isOkay);
            Logger.recordOutput("AutoShoot/desiredSpeed", parameters.desiredSpeed());
            Logger.recordOutput("AutoShoot/hoodAngleDeg",
                MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0));
            Logger.recordOutput("AutoShoot/distanceFeet", Units.metersToFeet(distance));
        }, shooter, swerve);
    }

    /** Shoot at a given target. */
    public static Command shoot(RobotState state, Supplier<Translation2d> targetSupplier,
        Swerve swerve, Shooter shooter, Indexer indexer, AdjustableHood hood, Magazine magazine,
        DoubleSupplier adjustUp, DoubleSupplier adjustLeft, CommandXboxController controller,
        BooleanSupplier disableTurret) {
        return Commands.runEnd(() -> {
            var lookahead = state.getFieldRelativeSpeeds().times(0.05);
            final Translation2d target = targetSupplier.get()
                .plus(new Translation2d(lookahead.vxMetersPerSecond, lookahead.vyMetersPerSecond));
            Translation2d adjustedTarget = target;
            double turretFudge =
                swerve.state.getGlobalPoseEstimate().getRotation().getCos() < 0.5 ? 2 : 0;
            double adjustUpValue = Units.feetToMeters(adjustUp.getAsDouble() + turretFudge);
            Rotation2d adjustLeftValue = Rotation2d.fromDegrees(adjustLeft.getAsDouble());
            Logger.recordOutput("AutoShoot/AdjustUp", adjustUpValue);
            Logger.recordOutput("AutoShoot/AdjustLeft", adjustLeftValue);
            for (int i = 0; i < 20; i++) {
                double distance =
                    adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                        + adjustUpValue;
                var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                    shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), false);
                double tof = parameters.timeOfFlight();
                var forward = state.getFieldRelativeSpeeds().times(tof);
                adjustedTarget = target
                    .minus(new Translation2d(forward.vxMetersPerSecond, forward.vyMetersPerSecond));
            }
            Logger.recordOutput("AutoShoot/Target", target);
            Logger.recordOutput("AutoShoot/AdjustedTarget", adjustedTarget);
            Logger.recordOutput("AutoShoot/TargetDiff", adjustedTarget.minus(target));
            double distance =
                adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                    + adjustUpValue;
            var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), true);
            shooter.setVelocity(parameters.desiredSpeed());
            hood.setTargetAngle(
                Degrees.of(MathUtil.clamp(parameters.hoodAngleDeg() + 1.0, 0.0, 30.0)));
            followHub(swerve, controller, adjustedTarget);


            boolean isOkay = parameters.isOkayToShoot();
            Logger.recordOutput("AutoShoot/isOkay", isOkay);
            Logger.recordOutput("AutoShoot/desiredSpeed", parameters.desiredSpeed());
            Logger.recordOutput("AutoShoot/hoodAngleDeg",
                MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0));
            Logger.recordOutput("AutoShoot/distanceFeet", Units.metersToFeet(distance));
            if (isOkay) {
                magazine.setVoltage(3.0);
                indexer.setSpindexerDutyCycle(6.0);
            } else {
                magazine.setVoltage(0.0);
                indexer.setSpindexerDutyCycle(0.0);
            }
        }, () -> {
            shooter.setVelocity(0.0);
            magazine.setVoltage(0.0);
            indexer.setSpindexerDutyCycle(0.0);
        }, shooter, swerve, indexer, hood);
    }

    /** Point turret at hub. */
    public static Command followHub(Swerve swerve, CommandXboxController controller,
        Translation2d target) {
        return swerve.driveUserRelative(() -> {
            double vx = -controller.getLeftY() * Constants.Swerve.maxSpeed;
            double vy = -controller.getLeftX() * Constants.Swerve.maxSpeed;

            Rotation2d angleToHub = new Rotation2d(
                Radians.of(target.minus(swerve.state.getGlobalPoseEstimate().getTranslation())
                    .getAngle().getRadians()));
            Rotation2d currentRotation = swerve.state.getGlobalPoseEstimate().getRotation();
            double rotationError = angleToHub.minus(currentRotation).getRadians();
            double omega = rotationError * 5.0;
            omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                Math.min(Constants.Swerve.maxAngularVelocity, omega));
            ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);
            return fieldRelative;
        });
    }
}
