package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.EnumSet;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourContext;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;

public final class Rebuilt2026Reasoner
    implements Reasoner<BehaviourFlag, BehaviourContext>, AutoCloseable {
  private static final SignalKey<Boolean> WANT_DEFENSE = ReasoningKeys.boolKey("want_defense");
  private static final SignalKey<Boolean> WANT_AUTOPATH = ReasoningKeys.boolKey("want_autopath");
  private static final SignalKey<Boolean> WANT_SHUTTLE = ReasoningKeys.boolKey("want_shuttle");
  private static final SignalKey<Boolean> TESTING = ReasoningKeys.boolKey("testing");

  private static final int PH_SHUTTLE = 0;
  private static final int PH_AUTOPATH = 1;
  private static final int PH_DEFENSE = 2;

  private final NetworkTablesSignals nt;
  private final SequenceReasoner<BehaviourFlag, BehaviourContext> seq;

  public Rebuilt2026Reasoner() {
    this(NetworkTableInstance.getDefault(), "/Repulsor/Reasoning");
  }

  public Rebuilt2026Reasoner(NetworkTableInstance inst, String basePath) {
    NetworkTablesSignals nts = new NetworkTablesSignals(inst, basePath);
    nts.register(ReasoningKeys.ENABLED, false);
    nts.register(ReasoningKeys.AUTO, false);
    nts.register(ReasoningKeys.TELEOP, false);
    nts.register(ReasoningKeys.ENDGAME, false);
    nts.register(WANT_DEFENSE, false);
    nts.register(WANT_AUTOPATH, false);
    nts.register(WANT_SHUTTLE, false);
    nts.register(TESTING, false); 
    this.nt = nts;

    Clock clock = new WpiClock();

    SequenceReasoner.Builder<BehaviourFlag, BehaviourContext> b =
        new SequenceReasoner.Builder<BehaviourFlag, BehaviourContext>(BehaviourFlag.class, clock, nt)
            .startAt(PH_SHUTTLE);

    b.addPhaseFor("shuttle_15s", EnumSet.of(BehaviourFlag.SHUTTLE_MODE), 15.0);
    b.addPhase("autopath", EnumSet.of(BehaviourFlag.AUTOPATH_MODE), 0.0, 1e9, PH_AUTOPATH);
    b.addPhase("defense", EnumSet.of(BehaviourFlag.DEFENSE_MODE), 0.0, 1e9, PH_DEFENSE);

    b.addTransition(
        PH_AUTOPATH,
        "autopath_to_defense_on_nt",
        100,
        0.25,
        (ctx, signals) -> signals.getOr(WANT_DEFENSE, false),
        PH_DEFENSE);

    b.addTransition(
        PH_SHUTTLE,
        "shuttle_to_defense_on_nt",
        110,
        0.0,
        (ctx, signals) -> signals.getOr(WANT_DEFENSE, false),
        PH_DEFENSE);

    b.addTransition(
        PH_DEFENSE,
        "defense_to_autopath_on_nt",
        90,
        0.25,
        (ctx, signals) -> signals.getOr(WANT_AUTOPATH, false),
        PH_AUTOPATH);

    b.addTransition(
        PH_AUTOPATH,
        "autopath_to_shuttle_on_nt",
        80,
        0.25,
        (ctx, signals) -> signals.getOr(WANT_SHUTTLE, false),
        PH_SHUTTLE);

    b.addTransition(
        PH_DEFENSE,
        "defense_to_shuttle_on_nt",
        80,
        0.25,
        (ctx, signals) -> signals.getOr(WANT_SHUTTLE, false),
        PH_SHUTTLE);

    b.addTransition(
        PH_AUTOPATH,
        "autopath_to_defense_endgame",
        95,
        0.25,
        (ctx, signals) -> {
          double t = DriverStation.getMatchTime();
          boolean endgame = t >= 0.0 && t <= 20.0;
          signals.put(ReasoningKeys.ENDGAME, endgame);
          return endgame;
        },
        PH_DEFENSE);

    this.seq = b.build();
  }

  public Signals signals() {
    return seq.signals();
  }

  public void setWantDefense(boolean v) {
    seq.signals().put(WANT_DEFENSE, v);
    seq.signals().flush();
  }

  public void setWantAutopath(boolean v) {
    seq.signals().put(WANT_AUTOPATH, v);
    seq.signals().flush();
  }

  public void setWantShuttle(boolean v) {
    seq.signals().put(WANT_SHUTTLE, v);
    seq.signals().flush();
  }

  public void setTesting(boolean v) {
    seq.signals().put(TESTING, v);
    seq.signals().flush();
  }

  @Override
  public EnumSet<BehaviourFlag> update(BehaviourContext ctx) {
    seq.signals().put(ReasoningKeys.ENABLED, DriverStation.isEnabled());
    seq.signals().put(ReasoningKeys.AUTO, DriverStation.isAutonomous());
    seq.signals().put(ReasoningKeys.TELEOP, DriverStation.isTeleop());

    if (seq.signals().getOr(TESTING, false)) {
      EnumSet<BehaviourFlag> out = EnumSet.of(BehaviourFlag.AUTOPATH_MODE);
      seq.signals().flush();
      return out;
    }

    EnumSet<BehaviourFlag> out = seq.update(ctx);
    seq.signals().flush();
    return out;
  }

  @Override
  public void reset() {
    seq.reset();
    seq.signals().flush();
  }

  @Override
  public void close() {
    nt.close();
  }
}
