package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sim.PhysicsSim;
import frc.sim.COTS;
import frc.sim.config.DriveTrainSimulationConfig;
import frc.sim.physics.swerve.SwerveDrivetrainSim;

import edu.wpi.first.math.util.Units;

public class DrivetrainSimSubsystem extends SubsystemBase {

    private final SwerveDrivetrainSim swerveSim;

    public DrivetrainSimSubsystem() {
        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(COTS.ofMark4(
                edu.wpi.first.wpilibj.simulation.DCMotorSim.getNEO(1),    // Drive
                edu.wpi.first.wpilibj.simulation.DCMotorSim.getNEO550(1), // Steering
                COTS.WHEELS.COLSONS.cof,
                3 // L3 gearing
            ))
            .withTrackLengthTrackWidth(
                Units.inchesToMeters(24), Units.inchesToMeters(24))
            .withBumperSize(
                Units.inchesToMeters(31), Units.inchesToMeters(31)
            );

        swerveSim = new SwerveDrivetrainSim(config);
    }

    @Override
    public void simulationPeriodic() {
        // Update simulation
        swerveSim.update();

        // Optional: publish sim data to Shuffleboard/SmartDashboard
    }

    public SwerveDrivetrainSim getSim() {
        return swerveSim;
    }
}
