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

package org.curtinfrc.frc2026.util.Repulsor.Vision.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Random;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;

public final class AgentController {
  private final Random rng;
  private final List<Setpoints.GameSetpoint> scoreCycle;

  private final Pose2d collectA = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateBluePose();
  private final Pose2d collectB = Setpoints.Rebuilt2026.CENTER_COLLECT.approximateBluePose();

  private final double scorerSwapMin = 2.2;
  private final double scorerSwapMax = 4.2;
  private final double roamerSwapMin = 1.6;
  private final double roamerSwapMax = 3.5;
  private final double blockerSwapMin = 0.8;
  private final double blockerSwapMax = 1.6;

  private Translation2d lastScorerTarget = null;
  private Translation2d lastRoamerTarget = null;
  private Translation2d lastBlockerTarget = null;

  private double tScorer = 0;
  private double tRoamer = 0;
  private double tBlocker = 0;

  public AgentController(Random rng, List<Setpoints.GameSetpoint> scoreCycle) {
    this.rng = rng;
    this.scoreCycle = scoreCycle;
  }

  private Translation2d randomFieldPoint() {
    double x = 0.6 + rng.nextDouble() * (VisionSimWorld.FIELD_LENGTH - 1.2);
    double yMin = VisionSimWorld.FIELD_WIDTH * 0.15;
    double yMax = VisionSimWorld.FIELD_WIDTH * 0.85;
    double y = yMin + rng.nextDouble() * (yMax - yMin);
    return new Translation2d(x, y);
  }

  private Translation2d chooseScoreTarget() {
    return scoreCycle.get(rng.nextInt(scoreCycle.size())).approximateBluePose().getTranslation();
  }

  private Translation2d pickCollect(boolean preferA) {
    return (preferA ? collectA : collectB).getTranslation();
  }

  private Translation2d scorerTarget(boolean preferCollectA) {
    if (lastScorerTarget == null) {
      lastScorerTarget = pickCollect(preferCollectA);
      return lastScorerTarget;
    }

    boolean atCollect =
        lastScorerTarget.getDistance(collectA.getTranslation()) < 0.8
            || lastScorerTarget.getDistance(collectB.getTranslation()) < 0.8;

    lastScorerTarget = atCollect ? chooseScoreTarget() : pickCollect(preferCollectA);
    return lastScorerTarget;
  }

  private Translation2d roamerTarget() {
    lastRoamerTarget = randomFieldPoint();
    return lastRoamerTarget;
  }

  private Translation2d blockerTarget(VisionSimWorld.Agent a) {
    Pose2d self = VisionSimWorld.getSelfPose();
    if (self == null) {
      double laneX = VisionSimWorld.FIELD_LENGTH * (0.35 + 0.3 * rng.nextDouble());
      double laneY = VisionSimWorld.FIELD_WIDTH * (0.25 + 0.5 * rng.nextDouble());
      return new Translation2d(laneX, laneY);
    }
    Translation2d selfPos = self.getTranslation();
    Rotation2d selfYaw = self.getRotation();
    double look = 1.4 + 0.6 * rng.nextDouble();
    double lat = (rng.nextBoolean() ? +1 : -1) * (0.7 + 0.5 * rng.nextDouble());
    Translation2d fwd = new Translation2d(look, selfYaw);
    Translation2d side = new Translation2d(lat, selfYaw.rotateBy(Rotation2d.kCCW_90deg));
    return selfPos.plus(fwd).plus(side);
  }

  public Translation2d nextTarget(VisionSimWorld.Agent a, double dt) {
    switch (a.role) {
      case SCORER -> {
        tScorer += dt;
        double lim = scorerSwapMin + rng.nextDouble() * (scorerSwapMax - scorerSwapMin);
        if (tScorer >= lim) {
          tScorer = 0;
          return scorerTarget(a.team == 0);
        }
        return lastScorerTarget == null ? scorerTarget(a.team == 0) : lastScorerTarget;
      }
      case ROAMER -> {
        tRoamer += dt;
        double lim = roamerSwapMin + rng.nextDouble() * (roamerSwapMax - roamerSwapMin);
        if (tRoamer >= lim) {
          tRoamer = 0;
          return roamerTarget();
        }
        return lastRoamerTarget == null ? roamerTarget() : lastRoamerTarget;
      }
      default -> {
        tBlocker += dt;
        double lim = blockerSwapMin + rng.nextDouble() * (blockerSwapMax - blockerSwapMin);
        if (tBlocker >= lim) {
          tBlocker = 0;
          lastBlockerTarget = blockerTarget(a);
        }
        if (lastBlockerTarget == null) {
          lastBlockerTarget = blockerTarget(a);
        }
        return lastBlockerTarget;
      }
    }
  }

  public void step(VisionSimWorld.Agent a, Translation2d target, double dt) {
    double[] acc = a.ou.step(dt);
    double tx = target.getX();
    double ty = target.getY();
    double dx = tx - a.x;
    double dy = ty - a.y;
    double d = Math.hypot(dx, dy);
    double kP = 1.4;
    double ax = acc[0] + (d > 1e-3 ? kP * dx / Math.max(0.3, d) : 0.0);
    double ay = acc[1] + (d > 1e-3 ? kP * dy / Math.max(0.3, d) : 0.0);
    ax += wallForce(a.x, 0, VisionSimWorld.FIELD_LENGTH);
    ay += wallForce(a.y, 0, VisionSimWorld.FIELD_WIDTH);

    Pose2d self = VisionSimWorld.getSelfPose();
    double minDist = 0.8;
    if (self != null) {
      Translation2d selfPos = self.getTranslation();
      double sx = selfPos.getX();
      double sy = selfPos.getY();
      double nvx = a.vx + ax * dt;
      double nvy = a.vy + ay * dt;
      double nx = a.x + nvx * dt;
      double ny = a.y + nvy * dt;
      double nextDist = Math.hypot(nx - sx, ny - sy);
      if (nextDist < minDist) {
        double tx2 = -(ny - sy);
        double ty2 = nx - sx;
        double tNorm = Math.hypot(tx2, ty2);
        if (tNorm < 1e-4) {
          double angle = 2.0 * Math.PI * rng.nextDouble();
          tx2 = Math.cos(angle);
          ty2 = Math.sin(angle);
          tNorm = 1.0;
        }
        tx2 /= tNorm;
        ty2 /= tNorm;
        double mag = VisionSimWorld.MAX_ACCEL * 0.8;
        ax = tx2 * mag;
        ay = ty2 * mag;
      }
    }

    double aMag = Math.hypot(ax, ay);
    if (aMag > VisionSimWorld.MAX_ACCEL) {
      ax *= VisionSimWorld.MAX_ACCEL / aMag;
      ay *= VisionSimWorld.MAX_ACCEL / aMag;
    }

    a.vx += ax * dt;
    a.vy += ay * dt;

    double vMag = Math.hypot(a.vx, a.vy);
    if (vMag > VisionSimWorld.MAX_SPEED) {
      a.vx *= VisionSimWorld.MAX_SPEED / vMag;
      a.vy *= VisionSimWorld.MAX_SPEED / vMag;
    }

    a.x += a.vx * dt;
    a.y += a.vy * dt;

    a.x = VisionSimWorld.clamp(a.x, 0.05, VisionSimWorld.FIELD_LENGTH - 0.05);
    a.y = VisionSimWorld.clamp(a.y, 0.05, VisionSimWorld.FIELD_WIDTH - 0.05);

    if (self != null) {
      Translation2d selfPos = self.getTranslation();
      double sx = selfPos.getX();
      double sy = selfPos.getY();
      double dxs = a.x - sx;
      double dys = a.y - sy;
      double dist = Math.hypot(dxs, dys);
      if (dist < minDist) {
        double nx;
        double ny;
        if (dist < 1e-4) {
          double angle = 2.0 * Math.PI * rng.nextDouble();
          nx = Math.cos(angle);
          ny = Math.sin(angle);
        } else {
          nx = dxs / dist;
          ny = dys / dist;
        }
        double push = minDist - dist;
        a.x += nx * push;
        a.y += ny * push;
        a.x = VisionSimWorld.clamp(a.x, 0.05, VisionSimWorld.FIELD_LENGTH - 0.05);
        a.y = VisionSimWorld.clamp(a.y, 0.05, VisionSimWorld.FIELD_WIDTH - 0.05);
      }
    }

    vMag = Math.hypot(a.vx, a.vy);
    double minSpeed = 0.05 * VisionSimWorld.MAX_SPEED;
    if (vMag < minSpeed) {
      double angle = 2.0 * Math.PI * rng.nextDouble();
      double kick = 0.3 * VisionSimWorld.MAX_SPEED;
      a.vx = Math.cos(angle) * kick;
      a.vy = Math.sin(angle) * kick;
      vMag = kick;
    }

    if (vMag > 1e-3) {
      double targetYaw = Math.atan2(a.vy, a.vx);
      a.heading = VisionSimWorld.lerpAngle(a.heading, targetYaw, VisionSimWorld.HEADING_LERP);
    }
  }

  private double wallForce(double p, double min, double max) {
    double f = 0.0;
    double dMin = p - min;
    double dMax = max - p;
    if (dMin < VisionSimWorld.WALL_MARGIN) {
      double s = VisionSimWorld.WALL_MARGIN - Math.max(dMin, 1e-3);
      f += VisionSimWorld.WALL_GAIN * (s / (dMin + 0.1));
    }
    if (dMax < VisionSimWorld.WALL_MARGIN) {
      double s = VisionSimWorld.WALL_MARGIN - Math.max(dMax, 1e-3);
      f -= VisionSimWorld.WALL_GAIN * (s / (dMax + 0.1));
    }
    return f;
  }
}
