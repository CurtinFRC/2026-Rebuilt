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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;

final class ReactiveBypassVibrationTracker {
  private final ArrayDeque<ReactiveBypassSample> vib = new ArrayDeque<>();
  private double vibAccumTime = 0.0;

  void reset() {
    vib.clear();
    vibAccumTime = 0.0;
  }

  void feedWindow(Pose2d pose, Rotation2d heading, double dt, ReactiveBypassConfig cfg) {
    Translation2d p = pose.getTranslation();
    double cos = Math.cos(heading.getRadians());
    double sin = Math.sin(heading.getRadians());
    double sPara = p.getX() * cos + p.getY() * sin;
    double sPerp = -p.getX() * sin + p.getY() * cos;
    int signPara = 0;
    int signPerp = 0;
    if (!vib.isEmpty()) {
      ReactiveBypassSample last = vib.getLast();
      double dPara = sPara - last.sPara;
      double dPerp = sPerp - last.sPerp;
      if (Math.abs(dPara) > 1e-4) signPara = dPara > 0 ? +1 : -1;
      if (Math.abs(dPerp) > 1e-4) signPerp = dPerp > 0 ? +1 : -1;
    }
    vib.addLast(new ReactiveBypassSample(p, sPara, sPerp, signPara, signPerp, dt));
    vibAccumTime += dt;
    while (!vib.isEmpty() && vibAccumTime > cfg.vibWindowS) {
      vibAccumTime -= vib.removeFirst().dt;
    }
  }

  boolean isVibrating(ReactiveBypassConfig cfg) {
    if (vib.isEmpty()) return false;
    double pMin = vib.getFirst().sPara, pMax = pMin;
    double lMin = vib.getFirst().sPerp, lMax = lMin;
    int flipsPara = 0, flipsPerp = 0;
    int lastPara = 0, lastPerp = 0;
    for (ReactiveBypassSample s : vib) {
      pMin = Math.min(pMin, s.sPara);
      pMax = Math.max(pMax, s.sPara);
      lMin = Math.min(lMin, s.sPerp);
      lMax = Math.max(lMax, s.sPerp);
      if (s.signPara != 0 && lastPara != 0 && s.signPara != lastPara) flipsPara++;
      if (s.signPerp != 0 && lastPerp != 0 && s.signPerp != lastPerp) flipsPerp++;
      if (s.signPara != 0) lastPara = s.signPara;
      if (s.signPerp != 0) lastPerp = s.signPerp;
    }
    double dispPara = pMax - pMin;
    double dispPerp = lMax - lMin;
    double disp = Math.hypot(dispPara, dispPerp);
    int flips = Math.max(flipsPara, flipsPerp);
    return disp < cfg.vibMinDisp && flips >= cfg.vibMaxDirFlips;
  }

  boolean isForwardStuck(ReactiveBypassConfig cfg, double lastOcc) {
    if (vib.isEmpty()) return false;
    double requiredWindow = cfg.vibWindowS * cfg.stuckLookbackFrac;
    if (vibAccumTime < requiredWindow) return false;
    ReactiveBypassSample first = vib.getFirst();
    ReactiveBypassSample last = vib.getLast();
    double forwardDisp = Math.abs(last.sPara - first.sPara);
    return forwardDisp < cfg.stuckMinForwardProgress && lastOcc >= cfg.stuckOccMin;
  }

  boolean isVibratingWithOccBoost(ReactiveBypassConfig cfg, double lastOcc) {
    if (!isVibrating(cfg)) return false;
    return lastOcc >= Math.max(0.0, cfg.occHigh - cfg.escapeOccBoost);
  }
}
