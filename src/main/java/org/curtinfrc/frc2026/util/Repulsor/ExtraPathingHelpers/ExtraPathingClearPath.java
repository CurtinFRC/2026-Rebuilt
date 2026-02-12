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

package org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.SnowmanObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.TeardropObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner.VisionObstacle;

public final class ExtraPathingClearPath {
  private ExtraPathingClearPath() {}

  public static boolean isClearPath(
      String topicRoot,
      Translation2d start,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      boolean publishSamples) {
    final double eps = 1e-9;
    if (start.minus(goal).getNorm() < eps) {
      ExtraPathingRecording.recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
      return true;
    }

    if (ExtraPathingCollision.segmentCompletelyBlocked(
        start, goal, robotLengthMeters, robotWidthMeters, obstacles)) {
      ExtraPathingRecording.recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
      return false;
    }

    final double robotHalfDiag = Math.hypot(robotLengthMeters, robotWidthMeters) * 0.5;
    final double buffer = 0.2;
    final double corridorR = robotHalfDiag + buffer;

    final double GOAL_CAPTURE_RADIUS = 0.20;
    final double PUSH_MARGIN = 0.05;
    final Translation2d goalEff = ExtraPathingMath.trimEnd(start, goal, GOAL_CAPTURE_RADIUS);
    ExtraPathingRecording.recordCircle(topicRoot + "/Goal/Capture", goal, GOAL_CAPTURE_RADIUS, 40);

    final Translation2d seg = goalEff.minus(start);
    final double segLen = Math.max(seg.getNorm(), 1e-9);
    final double terminalT = Math.max(0.0, 1.0 - (GOAL_CAPTURE_RADIUS / segLen));

    ExtraPathingRecording.recordCorridor(topicRoot + "/Corridor", start, goal, corridorR);
    ExtraPathingRecording.recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
    if (publishSamples) {
      ExtraPathingRecording.renderObstacles(topicRoot + "/Obstacles", obstacles, corridorR);
      ExtraPathingRecording.recordForbiddenGrid(
          topicRoot + "/Forbidden",
          obstacles,
          robotLengthMeters,
          robotWidthMeters,
          0.25,
          goal,
          GOAL_CAPTURE_RADIUS);
    }

    double minX = Math.min(start.getX(), goalEff.getX()) - corridorR;
    double maxX = Math.max(start.getX(), goalEff.getX()) + corridorR;
    double minY = Math.min(start.getY(), goalEff.getY()) - corridorR;
    double maxY = Math.max(start.getY(), goalEff.getY()) + corridorR;
    Predicate<Translation2d> inAABB =
        p -> p.getX() >= minX && p.getX() <= maxX && p.getY() >= minY && p.getY() <= maxY;

    java.util.function.BiPredicate<Obstacle, Double> allowInWindow =
        (obs, penetration) ->
            ExtraPathingObstacleUtil.isPushableObstacle(obs) && penetration <= PUSH_MARGIN;

    BiFunction<Translation2d, Translation2d, Boolean> segClearAgainstAll =
        (a, b) -> {
          double lminX = Math.min(a.getX(), b.getX()) - corridorR;
          double lmaxX = Math.max(a.getX(), b.getX()) + corridorR;
          double lminY = Math.min(a.getY(), b.getY()) - corridorR;
          double lmaxY = Math.max(a.getY(), b.getY()) + corridorR;

          for (Obstacle obs : obstacles) {
            if (obs == null) continue;

            if (obs instanceof PointObstacle p) {
              double eff = p.radius + corridorR;
              if (p.loc.getX() >= lminX
                  && p.loc.getX() <= lmaxX
                  && p.loc.getY() >= lminY
                  && p.loc.getY() <= lmaxY) {
                ExtraPathingMath.DistParam dp = ExtraPathingMath.pointToSegDistParam(p.loc, a, b);
                if (dp.dist() <= eff) {
                  double pen = eff - dp.dist();
                  if (dp.t() <= terminalT) return false;
                  if (!allowInWindow.test(obs, pen)) return false;
                }
              }
              continue;
            }

            if (obs instanceof SnowmanObstacle s) {
              double eff = s.radius + corridorR;
              if (s.loc.getX() >= lminX
                  && s.loc.getX() <= lmaxX
                  && s.loc.getY() >= lminY
                  && s.loc.getY() <= lmaxY) {
                ExtraPathingMath.DistParam dp = ExtraPathingMath.pointToSegDistParam(s.loc, a, b);
                if (dp.dist() <= eff) {
                  double pen = eff - dp.dist();
                  if (dp.t() <= terminalT) return false;
                  if (!allowInWindow.test(obs, pen)) return false;
                }
              }
              continue;
            }

            if (obs instanceof HorizontalObstacle h) {
              double t = ExtraPathingMath.paramForYOnSegment(h.y, a, b);
              if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
                if (t <= terminalT) return false;
                return false;
              }
              double md = Math.min(Math.abs(a.getY() - h.y), Math.abs(b.getY() - h.y));
              if (md <= corridorR) return false;
              continue;
            }

            if (obs instanceof VerticalObstacle v) {
              double t = ExtraPathingMath.paramForXOnSegment(v.x, a, b);
              if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
                if (t <= terminalT) return false;
                return false;
              }
              double md = Math.min(Math.abs(a.getX() - v.x), Math.abs(b.getX() - v.x));
              if (md <= corridorR) return false;
              continue;
            }

            if (obs instanceof TeardropObstacle tdrop) {
              double eff = tdrop.primaryMaxRange + corridorR;
              ExtraPathingMath.DistParam dp = ExtraPathingMath.pointToSegDistParam(tdrop.loc, a, b);
              if (dp.dist() <= eff) {
                if (dp.t() <= terminalT) return false;
                return false;
              }
              Translation2d aa = a.minus(tdrop.loc);
              Translation2d bb = b.minus(tdrop.loc);
              double minXSeg = Math.min(aa.getX(), bb.getX());
              double maxXSeg = Math.max(aa.getX(), bb.getX());
              boolean xOverlap = maxXSeg >= 0.0 && minXSeg <= tdrop.tailLength;
              if (xOverlap) {
                double minLat = Math.min(Math.abs(aa.getY()), Math.abs(bb.getY()));
                if ((aa.getY() <= 0 && bb.getY() >= 0) || (aa.getY() >= 0 && bb.getY() <= 0))
                  minLat = 0.0;
                if (minLat <= tdrop.primaryMaxRange + corridorR) return false;
              }
              continue;
            }

            if (obs instanceof VisionObstacle vo) {
              double eff = Math.max(vo.sizeX, vo.sizeY) * 0.5 + corridorR;
              if (vo.loc.getX() >= lminX
                  && vo.loc.getX() <= lmaxX
                  && vo.loc.getY() >= lminY
                  && vo.loc.getY() <= lmaxY) {
                ExtraPathingMath.DistParam dp = ExtraPathingMath.pointToSegDistParam(vo.loc, a, b);
                if (dp.dist() <= eff) {
                  double pen = eff - dp.dist();
                  if (dp.t() <= terminalT) return false;
                  if (!allowInWindow.test(obs, pen)) return false;
                }
              }
              continue;
            }

            return false;
          }
          return true;
        };

    boolean directClear = true;
    for (Obstacle obs : obstacles) {
      if (obs == null) continue;

      if (obs instanceof PointObstacle p) {
        double eff = p.radius + corridorR;
        if (inAABB.test(p.loc)) {
          ExtraPathingMath.DistParam dp =
              ExtraPathingMath.pointToSegDistParam(p.loc, start, goalEff);
          if (dp.dist() <= eff) {
            double pen = eff - dp.dist();
            if (dp.t() <= terminalT) {
              directClear = false;
              break;
            }
            if (!allowInWindow.test(obs, pen)) {
              directClear = false;
              break;
            }
          }
        }
        continue;
      }

      if (obs instanceof SnowmanObstacle s) {
        double eff = s.radius + corridorR;
        if (inAABB.test(s.loc)) {
          ExtraPathingMath.DistParam dp =
              ExtraPathingMath.pointToSegDistParam(s.loc, start, goalEff);
          if (dp.dist() <= eff) {
            double pen = eff - dp.dist();
            if (dp.t() <= terminalT) {
              directClear = false;
              break;
            }
            if (!allowInWindow.test(obs, pen)) {
              directClear = false;
              break;
            }
          }
        }
        continue;
      }

      if (obs instanceof HorizontalObstacle h) {
        double t = ExtraPathingMath.paramForYOnSegment(h.y, start, goalEff);
        if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
          directClear = false;
          break;
        }
        double md = Math.min(Math.abs(start.getY() - h.y), Math.abs(goalEff.getY() - h.y));
        if (md <= corridorR) {
          directClear = false;
          break;
        }
        continue;
      }

      if (obs instanceof VerticalObstacle v) {
        double t = ExtraPathingMath.paramForXOnSegment(v.x, start, goalEff);
        if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
          directClear = false;
          break;
        }
        double md = Math.min(Math.abs(start.getX() - v.x), Math.abs(goalEff.getX() - v.x));
        if (md <= corridorR) {
          directClear = false;
          break;
        }
        continue;
      }

      if (obs instanceof TeardropObstacle tdrop) {
        double eff = tdrop.primaryMaxRange + corridorR;
        ExtraPathingMath.DistParam dp =
            ExtraPathingMath.pointToSegDistParam(tdrop.loc, start, goalEff);
        if (inAABB.test(tdrop.loc) && dp.dist() <= eff) {
          directClear = false;
          break;
        }
        Translation2d a = start.minus(tdrop.loc), b = goalEff.minus(tdrop.loc);
        double minXSeg = Math.min(a.getX(), b.getX()), maxXSeg = Math.max(a.getX(), b.getX());
        boolean xOverlap = maxXSeg >= 0.0 && minXSeg <= tdrop.tailLength;
        if (xOverlap) {
          double minLat = Math.min(Math.abs(a.getY()), Math.abs(b.getY()));
          if ((a.getY() <= 0 && b.getY() >= 0) || (a.getY() >= 0 && b.getY() <= 0)) minLat = 0.0;
          if (minLat <= tdrop.primaryMaxRange + corridorR) {
            directClear = false;
            break;
          }
        }
        continue;
      }

      if (obs instanceof VisionObstacle vo) {
        double eff = Math.max(vo.sizeX, vo.sizeY) * 0.5 + corridorR;
        if (inAABB.test(vo.loc)) {
          ExtraPathingMath.DistParam dp =
              ExtraPathingMath.pointToSegDistParam(vo.loc, start, goalEff);
          if (dp.dist() <= eff) {
            double pen = eff - dp.dist();
            if (dp.t() <= terminalT) {
              directClear = false;
              break;
            }
            if (!allowInWindow.test(obs, pen)) {
              directClear = false;
              break;
            }
          }
        }
        continue;
      }

      directClear = false;
      break;
    }

    if (directClear) {
      return true;
    }

    List<Translation2d> candidates = new ArrayList<>();
    Translation2d dir = goal.minus(start);
    double dirN = dir.getNorm();
    if (dirN > eps) {
      Translation2d u = new Translation2d(dir.getX() / dirN, dir.getY() / dirN);
      Translation2d n = new Translation2d(-u.getY(), u.getX());
      double detour = corridorR * 1.5;
      candidates.add(start.plus(n.times(detour)));
      candidates.add(start.minus(n.times(detour)));
      candidates.add(goal.plus(n.times(detour)));
      candidates.add(goal.minus(n.times(detour)));
    }

    for (Obstacle obs : obstacles) {
      if (obs instanceof PointObstacle p) {
        candidates.addAll(
            ExtraPathingMath.offsetAround(p.loc, p.radius + corridorR + 0.05, start, goal));
      } else if (obs instanceof SnowmanObstacle s) {
        candidates.addAll(
            ExtraPathingMath.offsetAround(s.loc, s.radius + corridorR + 0.05, start, goal));
      } else if (obs instanceof HorizontalObstacle h) {
        double y = h.y + Math.copySign(corridorR + 0.05, goal.getY() - start.getY());
        candidates.add(new Translation2d((start.getX() + goal.getX()) * 0.5, y));
      } else if (obs instanceof VerticalObstacle v) {
        double x = v.x + Math.copySign(corridorR + 0.05, goal.getX() - start.getX());
        candidates.add(new Translation2d(x, (start.getY() + goal.getY()) * 0.5));
      } else if (obs instanceof TeardropObstacle t) {
        Translation2d sg = goal.minus(start);
        double vn = sg.getNorm();
        if (vn > eps) {
          Translation2d u = new Translation2d(sg.getX() / vn, sg.getY() / vn);
          Translation2d n = new Translation2d(-u.getY(), u.getX());
          double R = t.primaryMaxRange + corridorR + 0.1;
          candidates.add(t.loc.plus(n.times(R)));
          candidates.add(t.loc.minus(n.times(R)));
        }
      } else if (obs instanceof VisionObstacle vo) {
        double R = Math.max(vo.sizeX, vo.sizeY) * 0.5 + corridorR + 0.05;
        Translation2d sg = goal.minus(start);
        double vn = sg.getNorm();
        if (vn > eps) {
          Translation2d u = new Translation2d(sg.getX() / vn, sg.getY() / vn);
          Translation2d n = new Translation2d(-u.getY(), u.getX());
          candidates.add(vo.loc.plus(n.times(R)));
          candidates.add(vo.loc.minus(n.times(R)));
        }
      }
    }

    if (publishSamples) {
      ExtraPathingRecording.recordPoints(topicRoot + "/Candidates", candidates);
    }

    for (Translation2d w : candidates) {
      if (segClearAgainstAll.apply(start, w) && segClearAgainstAll.apply(w, goalEff)) {
        ExtraPathingRecording.recordPath(topicRoot + "/Path/Chosen", List.of(start, w, goal));
        return true;
      }
    }

    return false;
  }
}
