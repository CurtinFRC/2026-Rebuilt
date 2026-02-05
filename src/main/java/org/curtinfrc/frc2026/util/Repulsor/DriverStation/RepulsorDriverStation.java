/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import java.util.Objects;

public abstract class RepulsorDriverStation implements AutoCloseable {
  private static volatile RepulsorDriverStation INSTANCE;

  public static RepulsorDriverStation getInstance() {
    RepulsorDriverStation inst = INSTANCE;
    if (inst == null) throw new IllegalStateException("RepulsorDriverStation not initialized");
    return inst;
  }

  public static boolean isInitialized() {
    return INSTANCE != null;
  }

  public static void setInstance(RepulsorDriverStation instance) {
    Objects.requireNonNull(instance, "instance");
    RepulsorDriverStation prev;
    synchronized (RepulsorDriverStation.class) {
      prev = INSTANCE;
      INSTANCE = instance;
    }
    if (prev != null && prev != instance) prev.close();
  }

  public static void clearInstance() {
    RepulsorDriverStation prev;
    synchronized (RepulsorDriverStation.class) {
      prev = INSTANCE;
      INSTANCE = null;
    }
    if (prev != null) prev.close();
  }

  public abstract void tick();

  @Override
  public abstract void close();
}

