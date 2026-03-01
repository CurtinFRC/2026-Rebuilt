package org.curtinfrc.frc2026.util.Repulsor;

public class StaticInstance {
    public static Repulsor repulsor;

      public static Repulsor getInstance() {
    Repulsor local = repulsor;
    if (local == null) {
      synchronized (Repulsor.class) {
        local = repulsor;
        if (local == null) {
            throw new IllegalStateException("Repulsor instance has not been initialized. Please initialize it before calling getInstance().");
        }
      }
    }
    return local;
  }

  public static void initialize(Repulsor repulsorInstance) {
    synchronized (Repulsor.class) {
      if (repulsor == null) {
        repulsor = repulsorInstance;
      } else {
        throw new IllegalStateException("Repulsor instance has already been initialized.");
      }
    }
  }
}
