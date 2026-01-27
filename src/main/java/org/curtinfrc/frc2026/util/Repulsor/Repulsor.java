package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.AutoPathBehaviour;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourManager;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.DefenseBehaviour;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.ShuttleBehaviour;
import org.curtinfrc.frc2026.util.Repulsor.Commands.Triggers;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.FieldVision;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Flags.FlagManager;
import org.curtinfrc.frc2026.util.Repulsor.Reasoning.Rebuilt2026Reasoner;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuningHeat;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;
import org.littletonrobotics.junction.Logger;

public class Repulsor {

  public enum UsageType {
    kFullAuto,
    kAutoDrive
  }

  private double robot_x;
  private double robot_y;
  private double algae_offset;
  private double coral_offset;

  private Supplier<Double> shooterReleaseHeightMeters = () -> 0.0;

  private FieldPlanner m_planner;
  private VisionPlanner m_visionPlanner = new VisionPlanner();
  private DriveRepulsor m_drive;
  private UsageType m_usageType = UsageType.kAutoDrive;

  private RepulsorSetpoint m_currentGoal =
      new RepulsorSetpoint(Setpoints.Rebuilt2026.CENTER_COLLECT, HeightSetpoint.NONE);

  private RepulsorSetpoint m_nextScore =
      new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);

  private final List<FieldVision> m_fieldVisions = new ArrayList<>();

  private Optional<Trigger> m_gateInScoring = Optional.empty();
  private Optional<Trigger> m_gateInCollecting = Optional.empty();

  private Supplier<Boolean> m_hasPiece = () -> false;

  private BehaviourManager m_behaviourManager;
  private FlagManager<BehaviourFlag> m_behaviourFlags;

  public boolean atSetpoint() {
    Optional<Distance> err = m_planner.getErr();
    if (err.isEmpty()) return false;
    Logger.recordOutput("Repulsor/err", err.get());
    return err.isPresent() && err.get().lt(Meters.of(0.1));
  }

  public Repulsor withHasPieceSupplier(Supplier<Boolean> hasPiece) {
    this.m_hasPiece = hasPiece;
    return this;
  }

  public Repulsor withShooterReleaseHeightMetersSupplier(Supplier<Double> supplier) {
    this.shooterReleaseHeightMeters = supplier == null ? () -> 0.0 : supplier;
    return this;
  }

  public <T> Repulsor followGate(Triggers.PhaseGate<T> gate, T collecting, T scoring) {
    Trigger inScoring = gate.when(scoring);
    Trigger inCollecting = gate.when(collecting);

    m_gateInScoring = Optional.of(inScoring);
    m_gateInCollecting = Optional.of(inCollecting);
    return this;
  }

  public <E extends Enum<E>> Repulsor followGate(
      Triggers.ParallelGate<E> gate, EnumSet<E> collectingTags, EnumSet<E> scoringTags) {

    if (collectingTags.isEmpty() || scoringTags.isEmpty()) {
      throw new IllegalArgumentException("collectingTags and scoringTags must be non-empty");
    }

    Supplier<Boolean> scoringAllOn =
        () ->
            collectingTags.stream().allMatch(gate::isOn)
                && scoringTags.stream().allMatch(gate::isOn);
    Supplier<Boolean> collectingAllOn = () -> collectingTags.stream().allMatch(gate::isOn);

    Trigger inScoring = new Trigger(scoringAllOn::get);
    Trigger inCollecting = new Trigger(collectingAllOn::get);

    m_gateInScoring = Optional.of(inScoring);
    m_gateInCollecting = Optional.of(inCollecting);
    return this;
  }

  public Repulsor(
      DriveRepulsor drive,
      UsageType usageType,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      Supplier<Boolean> hasPiece) {
    this.m_drive = drive;
    this.m_usageType = usageType;
    this.robot_x = robot_x;
    this.robot_y = robot_y;
    this.coral_offset = coral_offset;
    this.algae_offset = algae_offset;
    this.m_hasPiece = hasPiece;

    m_planner = new FieldPlanner(new Rebuilt2026(), new DriveTuningHeat(() -> m_drive.getPose()));

    Supplier<Boolean> inScoreSup = () -> m_gateInScoring.map(Trigger::getAsBoolean).orElse(true);
    Supplier<Boolean> inCollectSup =
        () -> m_gateInCollecting.map(Trigger::getAsBoolean).orElse(false);
    Supplier<RepulsorSetpoint> nextScoreSup = () -> m_nextScore;

    Trigger atOutpost = within(Meters.of(0.45), Setpoints.Rebuilt2026.OUTPOST_COLLECT);
    Supplier<Boolean> atHPSup = atOutpost::getAsBoolean;

    Supplier<List<RepulsorSetpoint>> hpOptionsSup =
        () ->
            List.of(
                new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE));

    Supplier<RepulsorSetpoint> shuttleGoalSup =
        () -> new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE);

    Supplier<RepulsorSetpoint> defenseGoalSup =
        () -> new RepulsorSetpoint(Setpoints.Rebuilt2026.CENTER_COLLECT, HeightSetpoint.NONE);

    m_behaviourManager =
        new BehaviourManager()
            .add(new DefenseBehaviour(30, defenseGoalSup, () -> 2.8))
            .add(new ShuttleBehaviour(20, shuttleGoalSup, () -> 3.2))
            .add(
                new AutoPathBehaviour(
                    10,
                    inScoreSup,
                    inCollectSup,
                    nextScoreSup,
                    hpOptionsSup,
                    atHPSup,
                    m_hasPiece,
                    sp -> sp.point().name(),
                    () -> 3.5));

    Rebuilt2026Reasoner reasoner = new Rebuilt2026Reasoner();
    reasoner.setTesting(true);
    m_behaviourManager.setReasoner(reasoner);

    FieldTracker ft = FieldTracker.getInstance();
    FieldVision front = ft.new FieldVision("main");
    m_fieldVisions.add(front);
  }

  public Repulsor(
      DriveRepulsor drive,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      Supplier<Boolean> hasPiece) {
    this(drive, UsageType.kFullAuto, robot_x, robot_y, coral_offset, algae_offset, hasPiece);
  }

  public Repulsor withInitialNext(RepulsorSetpoint setpoint) {
    if (setpoint != null && setpoint.point().type() == SetpointType.kHumanPlayer) {
      throw new Error("Next score setpoint cannot be a human-player one");
    }
    m_nextScore = setpoint;
    return this;
  }

  public Repulsor withInitialHP(RepulsorSetpoint setpoint) {
    if (setpoint != null && setpoint.point().type() != SetpointType.kHumanPlayer) {
      throw new Error("Next collect setpoint must be a human-player/collect one");
    }
    m_nextScore = setpoint;
    return this;
  }

  public void setNextScore(RepulsorSetpoint next) {
    if (next != null && next.point().type() == SetpointType.kHumanPlayer) {
      throw new Error("Next score setpoint cannot be a human-player one");
    }
    m_nextScore = next;
  }

  public Repulsor withFallback(PlannerFallback fallback) {
    m_planner = m_planner.withFallback(fallback);
    return this;
  }

  public Repulsor withVision(RepulsorVision vision) {
    m_visionPlanner.addVision(vision);
    return this;
  }

  public Repulsor withFieldVision(FieldVision vision) {
    m_fieldVisions.add(vision);
    return this;
  }

  public void addFieldVision(FieldVision vision) {
    m_fieldVisions.add(vision);
  }

  public FieldPlanner getFieldPlanner() {
    return m_planner;
  }

  public VisionPlanner getVisionPlanner() {
    return m_visionPlanner;
  }

  public void update() {
    for (var vision : m_fieldVisions) {
      vision.update(m_drive.getPose());
    }

    refreshNextScoreFromFieldTracker();

    m_visionPlanner.tick();

    boolean enabled = true;
    RepulsorDriverStation dsBase = RepulsorDriverStation.getInstance();
    if (dsBase instanceof NtRepulsorDriverStation ds) {
      enabled = ds.getConfigBool("force_controller_override");
    }

    if (!enabled) {
      m_behaviourManager.stop();
      return;
    }
    
    m_behaviourManager.update(
        new BehaviourContext(
            this,
            m_planner,
            m_visionPlanner,
            m_drive,
            robot_x,
            robot_y,
            coral_offset,
            algae_offset,
            m_drive::getPose));
  }

  public DriveRepulsor getDrive() {
    return m_drive;
  }

  public void setup() {
    if (m_usageType != UsageType.kFullAuto) return;
  }

  private SetpointContext ctxFor(Pose2d robotPose, List<? extends FieldPlanner.Obstacle> dyn) {
    double len = Math.max(0.0, robot_x) * 2.0;
    double wid = Math.max(0.0, robot_y) * 2.0;
    double release =
        shooterReleaseHeightMeters == null ? 0.0 : Math.max(0.0, shooterReleaseHeightMeters.get());
    return new SetpointContext(
        Optional.ofNullable(robotPose), len, wid, coral_offset, algae_offset, release, dyn);
  }

  private Command alignCore(
      Supplier<RepulsorSetpoint> supplier,
      Optional<Trigger> untilOpt,
      CategorySpec cat,
      boolean suppressFallback) {
    final AtomicReference<RepulsorSetpoint> activeRef = new AtomicReference<>();
    final AtomicBoolean initialized = new AtomicBoolean(false);

    Command cmd =
        Commands.run(
                () -> {
                  if (!initialized.get()) {
                    activeRef.set(supplier.get());
                    initialized.set(true);
                  }

                  m_planner.pollChosenSetpoint().ifPresent(activeRef::set);

                  RepulsorSetpoint effective = activeRef.get();
                  if (effective == null) return;

                  m_currentGoal = effective;

                  Pose2d robotPose = m_drive.getPose();
                  List<? extends FieldPlanner.Obstacle> dyn = m_visionPlanner.getObstacles();
                  Pose2d goalPose = effective.get(ctxFor(robotPose, dyn));
                  m_planner.setGoal(goalPose);

                  RepulsorSample sample =
                      m_planner.calculate(
                          robotPose,
                          dyn,
                          robot_x,
                          robot_y,
                          coral_offset,
                          algae_offset,
                          cat,
                          suppressFallback,
                          shooterReleaseHeightMeters == null
                              ? 0.0
                              : Math.max(0.0, shooterReleaseHeightMeters.get()));

                  m_planner
                      .pollChosenSetpoint()
                      .ifPresent(
                          sp -> {
                            activeRef.set(sp);
                            m_currentGoal = sp;
                            Pose2d g = sp.get(ctxFor(robotPose, dyn));
                            m_planner.setGoal(g);
                          });

                  m_drive.runVelocity(
                      sample.asChassisSpeeds(m_drive.getOmegaPID(), robotPose.getRotation()));
                },
                m_drive)
            .finallyDo(interrupted -> m_drive.runVelocity(new ChassisSpeeds()));

    if (untilOpt.isPresent()) {
      cmd = cmd.until(untilOpt.get());
    }
    return cmd;
  }

  public Command alignTo(Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat) {
    return alignCore(point, Optional.of(until), cat, false);
  }

  public Command alignTo(RepulsorSetpoint point, Trigger until, CategorySpec cat) {
    return alignCore(() -> point, Optional.of(until), cat, false);
  }

  public Command alignTo(RepulsorSetpoint point, CategorySpec cat) {
    return alignCore(() -> point, Optional.empty(), cat, false);
  }

  public Command alignTo(Supplier<RepulsorSetpoint> point, CategorySpec cat) {
    return alignCore(point, Optional.empty(), cat, false);
  }

  public Command alignTo(
      Supplier<RepulsorSetpoint> point, Trigger until, CategorySpec cat, boolean suppressFallback) {
    return alignCore(point, Optional.of(until), cat, suppressFallback);
  }

  public Command alignTo(
      RepulsorSetpoint point, Trigger until, CategorySpec cat, boolean suppressFallback) {
    return alignCore(() -> point, Optional.of(until), cat, suppressFallback);
  }

  public Command alignTo(RepulsorSetpoint point, CategorySpec cat, boolean suppressFallback) {
    return alignCore(() -> point, Optional.empty(), cat, suppressFallback);
  }

  public Command alignTo(
      Supplier<RepulsorSetpoint> point, CategorySpec cat, boolean suppressFallback) {
    return alignCore(point, Optional.empty(), cat, suppressFallback);
  }

  public void setCurrentGoal(RepulsorSetpoint sp) {
    m_currentGoal = sp;
  }

  public HeightSetpoint getTargetHeight() {
    return m_currentGoal == null ? HeightSetpoint.NONE : m_currentGoal.height();
  }

  private Trigger withinCore(
      Distance d, Optional<SetpointType> typeOpt, Optional<GameSetpoint> pointOpt) {
    return new Trigger(
        () -> {
          Optional<Distance> err = m_planner.getErr();
          if (err.isEmpty()) {
            return false;
          }
          boolean within = err.get().lt(d);
          if (typeOpt.isPresent()) {
            within =
                within && m_currentGoal != null && m_currentGoal.point().type() == typeOpt.get();
          }
          if (pointOpt.isPresent()) {
            within = within && m_currentGoal != null && m_currentGoal.point() == pointOpt.get();
          }
          return within;
        });
  }

  public Trigger within(Distance d) {
    return withinCore(d, Optional.empty(), Optional.empty());
  }

  public Trigger within(Distance d, SetpointType t) {
    return withinCore(d, Optional.of(t), Optional.empty());
  }

  public Trigger within(Distance d, GameSetpoint p) {
    return withinCore(d, Optional.empty(), Optional.of(p));
  }

  private Optional<RepulsorSetpoint> chooseNextScoreFromFieldTracker() {
    FieldTracker ft = FieldTracker.getInstance();
    GameElement[] elements = ft.getFieldMap();
    if (elements == null || elements.length == 0) return Optional.empty();

    Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return Optional.empty();
    Alliance preferred =
        allianceOpt.get() == DriverStation.Alliance.Blue ? Alliance.kBlue : Alliance.kRed;

    for (GameElement e : elements) {
      if (e.getAlliance() == preferred && !e.isAtCapacity()) {
        Optional<RepulsorSetpoint> sp = e.getRelatedPoint();
        if (sp.isPresent()) return sp;
      }
    }

    return Optional.empty();
  }

  private void refreshNextScoreFromFieldTracker() {
    Optional<RepulsorSetpoint> ftNext = chooseNextScoreFromFieldTracker();
    ftNext.ifPresent(this::setNextScore);
  }
}
