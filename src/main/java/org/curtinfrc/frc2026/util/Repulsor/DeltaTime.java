package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.wpilibj.Timer;

public final class DeltaTime {
  private static final DeltaTime INSTANCE = new DeltaTime();

  public static DeltaTime getInstance() {
    return INSTANCE;
  }

  public static double get() {
    return INSTANCE._get();
  }

  public static void update() {
    INSTANCE._update();
  }

  private double lastTimeS = Double.NaN;
  private volatile double dtS = 0.0;

  private static final double MAX_DT_S = 0.1;

  private DeltaTime() {}

  private void _update() {
    final double nowS = Timer.getFPGATimestamp();

    if (Double.isNaN(lastTimeS)) {
      dtS = 0.0;
    } else {
      double dt = nowS - lastTimeS;

      if (dt < 0.0) dt = 0.0;

      if (dt > MAX_DT_S) dt = MAX_DT_S;

      dtS = dt;
    }

    lastTimeS = nowS;
  }

  private double _get() {
    return dtS;
  }
}
