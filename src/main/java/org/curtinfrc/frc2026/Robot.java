package org.curtinfrc.frc2026;

import static org.curtinfrc.frc2026.vision.Vision.compCameraConfigs;
import static org.curtinfrc.frc2026.vision.Vision.devCameraConfigs;

import choreo.auto.AutoFactory;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.curtinfrc.frc2026.drive.DevTunerConstants;
import org.curtinfrc.frc2026.drive.Drive;
import org.curtinfrc.frc2026.drive.GyroIO;
import org.curtinfrc.frc2026.drive.GyroIOPigeon2;
import org.curtinfrc.frc2026.drive.ModuleIO;
import org.curtinfrc.frc2026.drive.ModuleIOSim;
import org.curtinfrc.frc2026.drive.ModuleIOTalonFX;
import org.curtinfrc.frc2026.drive.TunerConstants;
import org.curtinfrc.frc2026.subsystems.Intake.Intake;
import org.curtinfrc.frc2026.subsystems.Intake.IntakeIOComp;
import org.curtinfrc.frc2026.subsystems.Intake.IntakeIODev;
import org.curtinfrc.frc2026.subsystems.Intake.IntakeIOSim;
import org.curtinfrc.frc2026.subsystems.Mag.Mag;
import org.curtinfrc.frc2026.subsystems.Mag.MagRoller.MagRollerIO;
import org.curtinfrc.frc2026.subsystems.Mag.MagRoller.MagRollerIOComp;
import org.curtinfrc.frc2026.subsystems.Mag.MagRoller.MagRollerIODev;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodIO;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodIOComp;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodIODev;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodIOSim;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.HoodedShooter;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.ShooterIO;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.ShooterIOComp;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.ShooterIODev;
import org.curtinfrc.frc2026.subsystems.hoodedshooter.ShooterIOSim;
import org.curtinfrc.frc2026.util.AutoChooser;
import org.curtinfrc.frc2026.util.FieldConstants;
import org.curtinfrc.frc2026.util.GameState;
import org.curtinfrc.frc2026.util.PhoenixUtil;
import org.curtinfrc.frc2026.util.VirtualSubsystem;
import org.curtinfrc.frc2026.vision.Vision;
import org.curtinfrc.frc2026.vision.VisionIO;
import org.curtinfrc.frc2026.vision.VisionIOPhotonVision;
import org.curtinfrc.frc2026.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Drive drive;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final Autos autos;
  private Vision vision;
  private Intake intake;
  private Mag mag;
  private HoodedShooter hoodedShooter;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Alert controllerDisconnected =
      new Alert("Driver controller disconnected!", AlertType.kError);

  @AutoLogOutput(key = "Triggers/IsRed")
  Trigger isRed = new Trigger(() -> DriverStation.getAlliance().get().equals(Alliance.Red));

  @AutoLogOutput(key = "Triggers/TrenchAlign")
  Trigger TrenchAlign = controller.leftBumper();

  @AutoLogOutput(key = "Triggers/IsBlue")
  Trigger isBlue = new Trigger(() -> DriverStation.getAlliance().get().equals(Alliance.Blue));

  @AutoLogOutput(key = "Triggers/IsLeft")
  Trigger isLeft =
      new Trigger(() -> drive.getPose().getY() < FieldConstants.Hub.topCenterPoint.getY());

  @AutoLogOutput(key = "Triggers/inOwnHalf")
  Trigger inOwnHalf =
      new Trigger(
          () -> {
            double robotX = drive.getPose().getX();
            double boundaryX =
                ChoreoAllianceFlipUtil.flip(FieldConstants.LeftTrench.openingTopLeft).getX();
            boolean isRedAlliance = true;
            // DriverStation.getAlliance().isPresent()
            //     && DriverStation.getAlliance().get() == Alliance.Red;
            return isRedAlliance ? robotX > boundaryX : robotX < boundaryX;
          });

  private boolean edge = true;

  @AutoLogOutput(key = "Triggers/Edging")
  Trigger edging = new Trigger(() -> edge);

  private boolean intaker = false;
  private boolean aligner = true;

  @AutoLogOutput(key = "Triggers/Intaking")
  Trigger intaking = new Trigger(() -> intaker);

  @AutoLogOutput(key = "Triggers/Aligning")
  Trigger aligning = new Trigger(() -> aligner);

  @AutoLogOutput(key = "Triggers/isInNeutral")
  Trigger isInNeutralZone =
      new Trigger(
          () -> {
            double robotX = drive.getPose().getX();
            double boundaryX =
                ChoreoAllianceFlipUtil.flip(FieldConstants.LeftTrench.openingTopLeft).getX();
            boolean isRedAlliance =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            return isRedAlliance ? robotX < boundaryX : robotX > boundaryX;
          });

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RobotType", Constants.robotType.toString());
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
      default -> Logger.recordMetadata("GitDirty", "Unknown");
    }

    switch (Constants.getMode()) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        Logger.addDataReceiver(new NT4Publisher());
      }

      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();
    SignalLogger.start();
    DataLogManager.start();

    DriverStation.waitForDsConnection(600);

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robotType) {
        case COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIOPhotonVision(
                      compCameraConfigs[0].name(), compCameraConfigs[0].robotToCamera()),
                  new VisionIOPhotonVision(
                      compCameraConfigs[1].name(), compCameraConfigs[1].robotToCamera()),
                  new VisionIOPhotonVision(
                      compCameraConfigs[2].name(), compCameraConfigs[2].robotToCamera()),
                  new VisionIOPhotonVision(
                      compCameraConfigs[3].name(), compCameraConfigs[3].robotToCamera()));
          hoodedShooter =
              new HoodedShooter(
                  new HoodIOComp() {},
                  new ShooterIOComp() {},
                  drive::getPose,
                  drive::getChassisSpeeds);
          intake = new Intake(new IntakeIOComp());
          mag =
              new Mag(
                  new MagRollerIOComp(
                      Constants.bBotIntakeMagRollerMotorID,
                      InvertedValue.CounterClockwise_Positive),
                  new MagRollerIOComp(
                      Constants.bBotIndexerMagRollerMotorID,
                      InvertedValue.CounterClockwise_Positive));
        }
        case DEV -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DevTunerConstants.FrontLeft),
                  new ModuleIOTalonFX(DevTunerConstants.FrontRight),
                  new ModuleIOTalonFX(DevTunerConstants.BackLeft),
                  new ModuleIOTalonFX(DevTunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIOPhotonVision(
                      devCameraConfigs[0].name(), devCameraConfigs[0].robotToCamera()),
                  new VisionIOPhotonVision(
                      devCameraConfigs[1].name(), devCameraConfigs[1].robotToCamera()),
                  new VisionIOPhotonVision(
                      devCameraConfigs[2].name(), devCameraConfigs[2].robotToCamera()),
                  new VisionIOPhotonVision(
                      devCameraConfigs[3].name(), devCameraConfigs[3].robotToCamera()));

          intake = new Intake(new IntakeIODev());
          mag =
              new Mag(
                  new MagRollerIODev(
                      Constants.alphaIntakeMagRollerMotorID,
                      InvertedValue.CounterClockwise_Positive),
                  new MagRollerIODev(
                      Constants.alphaMiddleMagRollerMotorID, InvertedValue.Clockwise_Positive),
                  new MagRollerIODev(
                      Constants.alphaIndexerMagRollerMotorID, InvertedValue.Clockwise_Positive));
          hoodedShooter =
              new HoodedShooter(
                  new HoodIODev(), new ShooterIODev(), drive::getPose, drive::getChassisSpeeds);
        }
        case SIM -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getRotation,
                  new VisionIOPhotonVisionSim(
                      compCameraConfigs[0].name(),
                      compCameraConfigs[0].robotToCamera(),
                      drive::getPose),
                  new VisionIOPhotonVisionSim(
                      compCameraConfigs[1].name(),
                      compCameraConfigs[1].robotToCamera(),
                      drive::getPose),
                  new VisionIOPhotonVisionSim(
                      compCameraConfigs[2].name(),
                      compCameraConfigs[2].robotToCamera(),
                      drive::getPose),
                  new VisionIOPhotonVisionSim(
                      compCameraConfigs[3].name(),
                      compCameraConfigs[3].robotToCamera(),
                      drive::getPose));
          mag = new Mag(new MagRollerIO() {}, new MagRollerIO() {}, new MagRollerIO() {});
          intake = new Intake(new IntakeIOSim());
          hoodedShooter =
              new HoodedShooter(
                  new HoodIOSim(), new ShooterIOSim(), drive::getPose, drive::getChassisSpeeds);
        }
      }
    } else {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      vision = new Vision(drive::addVisionMeasurement, drive::getRotation, new VisionIO() {});
      mag = new Mag(new MagRollerIO() {}, new MagRollerIO() {}, new MagRollerIO() {});
      hoodedShooter =
          new HoodedShooter(
              new HoodIO() {}, new ShooterIO() {}, drive::getPose, drive::getChassisSpeeds);
    }

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
    autoChooser = new AutoChooser();
    autos = new Autos(autoFactory, drive, intake, hoodedShooter, mag);

    autoChooser.addCmd("Left then forward", autos::testDrive);
    autoChooser.addCmd("Left trench, half", autos::leftHalfAuto);
    autoChooser.addCmd("Left trench", autos::leftTrench);

    autoChooser.addRoutine("Left trench, half routine", autos::leftHalfAutoRoutine);

    RobotModeTriggers.autonomous().whileTrue((autoChooser.selectedCommandScheduler()));

    CommandScheduler.getInstance().onCommandInitialize(this::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(this::commandEnded);
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> runningInterrupters.put(interrupter, interrupted));
              commandEnded(interrupted);
            });

    DriverStation.silenceJoystickConnectionWarning(true);

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    controller.a().whileTrue(drive.faceAngle(Rotation2d.kZero));
    controller.b().whileTrue(drive.faceAngle(Rotation2d.k180deg));

    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    inOwnHalf
        .and(RobotModeTriggers.teleop())
        .and(GameState.activeShift)
        .and(TrenchAlign.negate())
        .whileTrue(
            drive
                .locationHeadingjoyStickDrive(
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX(),
                    aligning,
                    () ->
                        hoodedShooter.getVirtualTargetLocation(
                            () ->
                                ChoreoAllianceFlipUtil.shouldFlip()
                                    ? ChoreoAllianceFlipUtil.flip(
                                        FieldConstants.Hub.topCenterPoint.toTranslation2d())
                                    : FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .withName("Scoring"));

    inOwnHalf
        .and(RobotModeTriggers.teleop())
        // .and(GameState.activeShift)
        .whileTrue(
            hoodedShooter.shootAtTarget(
                () ->
                    ChoreoAllianceFlipUtil.flip(
                        FieldConstants.Hub.topCenterPoint.toTranslation2d())));

    // isInNeutralZone
    //     .and(isLeft.negate())
    //     .and(TrenchAlign.negate())
    //     .and(GameState.activeShift.negate())
    //     .whileTrue(
    //         hoodedShooter.shootAtTarget(
    // () -> ChoreoAllianceFlipUtil.flip(FieldConstants.ShuttlePoint.ShuttlePointRight)));

    // isInNeutralZone
    //     .and(isLeft)
    //     .and(GameState.activeShift.negate())
    //     .and(TrenchAlign.negate())
    //     .whileTrue(
    //         hoodedShooter.shootAtTarget(
    //             () ->
    // ChoreoAllianceFlipUtil.flip(FieldConstants.ShuttlePoint.ShuttlePointLeft)));

    isLeft
        .negate()
        .and(isInNeutralZone)
        .and(TrenchAlign.negate())
        .whileTrue(
            drive
                .locationHeadingjoyStickDrive(
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX(),
                    aligning,
                    () ->
                        ChoreoAllianceFlipUtil.flip(FieldConstants.ShuttlePoint.ShuttlePointRight))
                .withName("RightShuttling"));

    isLeft
        .and(isInNeutralZone)
        .and(TrenchAlign.negate())
        .whileTrue(
            drive
                .locationHeadingjoyStickDrive(
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX(),
                    aligning,
                    () -> ChoreoAllianceFlipUtil.flip(FieldConstants.ShuttlePoint.ShuttlePointLeft))
                .withName("LeftShuttling"));

    controller.rightBumper().onTrue(Commands.runOnce(() -> intaker = !intaker));
    controller.leftTrigger().onTrue(Commands.runOnce(() -> aligner = !aligner));

    // TODO FIX
    RobotModeTriggers.disabled()
        .negate()
        .and(intaking)
        .whileTrue(intake.RawControlConsume(0.8))
        .onFalse(intake.RawIdle());
    RobotModeTriggers.disabled()
        .negate()
        .and(intaking)
        .whileTrue(mag.store(9.6))
        .onFalse(mag.store(0));
    hoodedShooter
        .hoodedShooterReady
        .whileTrue(mag.spinIndexer(9.6))
        .whileFalse(mag.holdIndexerCommand());

    controller
        .leftBumper()
        .whileTrue(drive.TrenchAlign(() -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // .whileTrue(
    //     Commands.runOnce(() -> edge = false)
    //         .andThen(
    //             drive.TrenchAlign(() -> -controller.getLeftY(), () -> -controller.getLeftX())
    //                 .finallyDo(() -> edge = true)));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    PhoenixUtil.refreshAll();
    VirtualSubsystem.periodicAll();
    GameState.periodic();
    CommandScheduler.getInstance().run();
    controllerDisconnected.set(!controller.isConnected());
    logRunningCommands();
    logRequiredSubsystems();
    Logger.recordOutput(
        "LoggedRobot/MemoryUsageMb",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6);
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoChooser.periodic();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private final Set<Command> runningNonInterrupters = new HashSet<>();
  private final Map<Command, Command> runningInterrupters = new HashMap<>();
  private final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  private void commandStarted(final Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningNonInterrupters.add(command);
    }

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  private void commandEnded(final Command command) {
    runningNonInterrupters.remove(command);
    runningInterrupters.remove(command);

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  private final StringBuilder subsystemsBuilder = new StringBuilder();

  private String getCommandName(Command command) {
    subsystemsBuilder.setLength(0);
    int j = 1;
    for (final Subsystem subsystem : command.getRequirements()) {
      subsystemsBuilder.append(subsystem.getName());
      if (j < command.getRequirements().size()) {
        subsystemsBuilder.append(",");
      }

      j++;
    }
    var finalName = command.getName();
    if (j > 1) {
      finalName += " (" + subsystemsBuilder + ")";
    }
    return finalName;
  }

  private void logRunningCommands() {
    Logger.recordOutput("CommandScheduler/Running/.type", "Alerts");

    final ArrayList<String> runningCommands = new ArrayList<>();
    final ArrayList<String> runningDefaultCommands = new ArrayList<>();
    for (final Command command : runningNonInterrupters) {
      boolean isDefaultCommand = false;
      for (Subsystem subsystem : command.getRequirements()) {
        if (subsystem.getDefaultCommand() == command) {
          runningDefaultCommands.add(getCommandName(command));
          isDefaultCommand = true;
          break;
        }
      }
      if (!isDefaultCommand) {
        runningCommands.add(getCommandName(command));
      }
    }
    Logger.recordOutput(
        "CommandScheduler/Running/warnings", runningCommands.toArray(new String[0]));
    Logger.recordOutput(
        "CommandScheduler/Running/infos", runningDefaultCommands.toArray(new String[0]));

    final String[] interrupters = new String[runningInterrupters.size()];
    int j = 0;
    for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
      final Command interrupter = entry.getKey();
      final Command interrupted = entry.getValue();

      interrupters[j] = getCommandName(interrupter) + " interrupted " + getCommandName(interrupted);
      j++;
    }

    Logger.recordOutput("CommandScheduler/Running/errors", interrupters);
  }

  private void logRequiredSubsystems() {
    Logger.recordOutput("CommandScheduler/Subsystems/.type", "Alerts");

    final String[] subsystems = new String[requiredSubsystems.size()];
    {
      int i = 0;
      for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
        final Subsystem required = entry.getKey();
        final Command command = entry.getValue();

        subsystems[i] = required.getName() + " (" + command.getName() + ")";
        i++;
      }
    }
    Logger.recordOutput("CommandScheduler/Subsystems/infos", subsystems);
  }
}
