package org.curtinfrc.frc2026.subsystems.hoodedshooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodedShooter extends SubsystemBase {
  public static final double WHEEL_DIAMETER = 0.101;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Alert hoodMotorDisconnectedAlert;
  private final Alert hoodMotorTempAlert;
  private final Alert[] shooterMotorDisconnectedAlerts = new Alert[3];
  private final Alert[] shooterMotorTempAlerts = new Alert[3];

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Radians.mutable(0);
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine;
  public static final double FORWARD_LIMIT = HoodIODev.FORWARD_LIMIT_ROTATIONS;
  public static final double REVERSE_LIMIT = HoodIODev.REVERSE_LIMIT_ROTATIONS;

  // Small buffer so we stop *before* slamming into the limit
  public static final double LIMIT_MARGIN = 0.02; // rotations

  public boolean atForwardLimit() {
    return hoodInputs.positionRotations >= (FORWARD_LIMIT - LIMIT_MARGIN);
  }

  public boolean atReverseLimit() {
    return hoodInputs.positionRotations <= (REVERSE_LIMIT + LIMIT_MARGIN);
  }

  public HoodedShooter(HoodIO hoodIO, ShooterIO shooterIO) {
    this.hoodIO = hoodIO;
    this.shooterIO = shooterIO;

    this.hoodMotorDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
    this.hoodMotorTempAlert =
        new Alert("Hood motor temperature above 60°C.", AlertType.kWarning); // change
    for (int motor = 0; motor < 3; motor++) {
      this.shooterMotorDisconnectedAlerts[motor] =
          new Alert("Shooter motor " + String.valueOf(motor) + " disconnected.", AlertType.kError);
      this.shooterMotorTempAlerts[motor] =
          new Alert(
              "Shooter motor " + String.valueOf(motor) + " temperature above 60°C.",
              AlertType.kWarning);
    }

    // m_sysIdRoutine =
    //     new SysIdRoutine(
    //         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    //         new SysIdRoutine.Config(),
    //         new SysIdRoutine.Mechanism(
    //             shooterIO::setVoltageV,
    //             log -> {
    //               // Record a frame for the shooter motor.
    //               log.motor("shooter-wheel")
    //                   .voltage(m_appliedVoltage.mut_replace(shooterInputs.appliedVolts, Volts))
    //                   .angularPosition(
    //                       m_angle.mut_replace(shooterInputs.positionRotations, Rotations))
    //                   .angularVelocity(
    //                       m_velocity.mut_replace(
    //                           shooterInputs.velocityMetresPerSecond
    //                               / (HoodedShooter.WHEEL_DIAMETER * Math.PI),
    //                           RotationsPerSecond));
    //             },
    //             this,
    //             "shooter"));

    m_sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                hoodIO::setVoltageV,
                log -> {
                  // Record a frame for the shooter motor.
                  log.motor("hood")
                      .voltage(m_appliedVoltage.mut_replace(hoodInputs.appliedVolts, Volts))
                      .angularPosition(m_angle.mut_replace(hoodInputs.positionRotations, Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              hoodInputs.angularVelocityRotationsPerSecond, RotationsPerSecond));
                },
                this,
                "hood"));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Hood", hoodInputs);
    Logger.processInputs("Shooter", shooterInputs);

    // Update alerts
    hoodMotorDisconnectedAlert.set(!hoodInputs.motorConnected);
    hoodMotorTempAlert.set(hoodInputs.motorTemperature > 60); // in celcius
    for (int motor = 0; motor < 3; motor++) {
      shooterMotorDisconnectedAlerts[motor].set(!shooterInputs.motorsConnected[motor]);
      shooterMotorTempAlerts[motor].set(shooterInputs.motorTemperatures[motor] > 60);
    }
  }

  public Command setHoodPosition(double position) {
    return run(() -> hoodIO.setPosition(position));
  }

  public Command stopHood() {
    return run(() -> hoodIO.setVoltage(0));
  }

  public Command setHoodVoltage(Supplier<Double> voltage) {
    return run(() -> hoodIO.setVoltage(voltage.get()));
  }

  public Command setShooterVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public Command stopShooter() {
    return run(() -> shooterIO.setVoltage(0));
  }

  public Command setShooterVelocity(double velocity) {
    return run(() -> shooterIO.setVelocity(velocity));
  }

  public Command sysIdQuasistaticForward() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(this::atForwardLimit);
  }

  public Command sysIdQuasistaticBackward() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(this::atReverseLimit);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamicForward() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(this::atForwardLimit);
  }

  public Command sysIdDynamicBackward() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(this::atReverseLimit);
  }

  public Command setHoodedShooterPositionAndVelocity(double position, double velocity) {
    return run(
        () -> {
          hoodIO.setPosition(position);
          shooterIO.setVelocity(velocity);
        });
  }

  public Command stopHoodedShooter() {
    return run(
        () -> {
          hoodIO.setVoltage(0);
          shooterIO.setVoltage(0);
        });
  }
}
