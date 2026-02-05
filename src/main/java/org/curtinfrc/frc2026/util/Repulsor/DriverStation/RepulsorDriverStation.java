/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
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

