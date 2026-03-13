package org.curtinfrc.frc2026;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.curtinfrc.frc2026.drive.Drive;
import org.curtinfrc.frc2026.subsystems.Intake.Intake;
import org.curtinfrc.frc2026.subsystems.Mag.Mag;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodedShooter;

public class Autos {
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final Intake intake;
  private final HoodedShooter hoodedShooter;
  private final Mag mag;

  public Autos(
      AutoFactory autoFactory, Drive drive, Intake intake, HoodedShooter hoodedShooter, Mag mag) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.intake = intake;
    this.hoodedShooter = hoodedShooter;
    this.mag = mag;
  }

  public Command leftHalfAuto() {
    return autoFactory
        .trajectoryCmd("leftHalfAuto")
        .andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
  }

  public Command leftTrench() {
    return autoFactory
        .trajectoryCmd("LeftTrench")
        .andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
  }

  public Command testDrive() {
    return autoFactory
        .trajectoryCmd("LeftForward")
        .andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
  }

  public AutoRoutine leftHalfAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("leftHalfAuto");
    AutoTrajectory leftHalfAuto = routine.trajectory("LeftHalfAuto");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // intake.RawControlConsume(4),
                // hoodedShooter.setHoodedShooterPositionAndVelocity(0.40, 18.2),
                // mag.spinIndexer(4),
                leftHalfAuto.cmd()));

    return routine;
  }

  // public Command leftFullAuto() {
  //   return autoFactory
  //       .trajectoryCmd("LeftFullAuto")
  //       .andThen(drive.joystickDrive(() -> 0, () -> 0, () -> 0));
  // }

  public AutoRoutine test1() {
    AutoRoutine routine = autoFactory.newRoutine("Straight Test");
    AutoTrajectory straightTraj = routine.trajectory("StraightLine");

    routine.active().onTrue(straightTraj.resetOdometry().andThen(straightTraj.cmd()));

    return routine;
  }

  public AutoRoutine test2() {
    AutoRoutine routine = autoFactory.newRoutine("Turn Test");
    AutoTrajectory straightTraj = routine.trajectory("SpinAuto");

    routine.active().onTrue(straightTraj.resetOdometry().andThen(straightTraj.cmd()));

    return routine;
  }

  public AutoRoutine rightHalfAutoRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("rightHalfAuto");
    AutoTrajectory rightHalfAuto = routine.trajectory("RightHalfAuto");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.RawControlConsume(4),
                hoodedShooter.setHoodedShooterPositionAndVelocity(0.40, 18.2),
                mag.spinIndexer(4),
                rightHalfAuto.cmd()));

    return routine;
  }

  public AutoRoutine leftFullAuto() {
    AutoRoutine routine = autoFactory.newRoutine("leftFullAuto");
    AutoTrajectory leftFullAuto = routine.trajectory("LeftFullAuto");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.run(() -> drive.setPose(leftFullAuto.getInitialPose().get()))
                    .withTimeout(0.2),
                Commands.parallel(
                    // intake.RawControlConsume(4),
                    // hoodedShooter.setHoodedShooterPositionAndVelocity(0.40, 18.2),
                    // mag.spinIndexer(4),
                    leftFullAuto.cmd())));

    return routine;
  }

  public AutoRoutine rightFullAuto() {
    AutoRoutine routine = autoFactory.newRoutine("rightFullAuto");
    AutoTrajectory rightFullAuto = routine.trajectory("RightFullAuto");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.RawControlConsume(4),
                hoodedShooter.setHoodedShooterPositionAndVelocity(0.40, 18.2),
                mag.spinIndexer(4),
                rightFullAuto.cmd()));

    return routine;
  }
}
