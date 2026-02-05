/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.BiFunction;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner.VisionObstacle;

public class ExtraPathing {
  private static void recordEllipse(
      String key, Translation2d c, double rx, double ry, int samples) {
    List<Translation2d> pts = new ArrayList<>(samples + 1);
    for (int i = 0; i <= samples; i++) {
      double th = 2.0 * Math.PI * i / samples;
      pts.add(new Translation2d(c.getX() + rx * Math.cos(th), c.getY() + ry * Math.sin(th)));
    }
    recordPath(key, pts);
  }

  private static Translation2d trimEnd(Translation2d a, Translation2d b, double trim) {
    Translation2d d = b.minus(a);
    double L = d.getNorm();
    if (L < 1e-9) return b;
    double keep = Math.max(0.0, L - trim);
    Translation2d u = new Translation2d(d.getX() / L, d.getY() / L);
    return a.plus(u.times(keep));
  }

  private static record DistParam(double dist, double t) {}

  private static DistParam pointToSegDistParam(Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double ab2 = ab.getNorm() * ab.getNorm();
    if (ab2 < 1e-12) return new DistParam(ap.getNorm(), 0.0);
    double t = Math.max(0.0, Math.min(1.0, FieldPlanner.dot(ap, ab) / ab2));
    Translation2d proj = a.plus(ab.times(t));
    return new DistParam(p.getDistance(proj), t);
  }

  private static double paramForYOnSegment(double y, Translation2d a, Translation2d b) {
    double dy = b.getY() - a.getY();
    if (Math.abs(dy) < 1e-12) return Double.NaN;
    return (y - a.getY()) / dy;
  }

  private static double paramForXOnSegment(double x, Translation2d a, Translation2d b) {
    double dx = b.getX() - a.getX();
    if (Math.abs(dx) < 1e-12) return Double.NaN;
    return (x - a.getX()) / dx;
  }

  private static boolean isPushableObstacle(FieldPlanner.Obstacle o) {
    return (o instanceof FieldPlanner.PointObstacle)
        || (o instanceof FieldPlanner.SnowmanObstacle)
        || (o instanceof VisionObstacle);
  }

  public static Translation2d[] rectCorners(Translation2d center, double length, double width) {
    double hx = length * 0.5, hy = width * 0.5;
    return new Translation2d[] {
      new Translation2d(center.getX() - hx, center.getY() - hy),
      new Translation2d(center.getX() + hx, center.getY() - hy),
      new Translation2d(center.getX() + hx, center.getY() + hy),
      new Translation2d(center.getX() - hx, center.getY() + hy)
    };
  }

  public static boolean robotIntersects(
      Translation2d center,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    Translation2d[] rect = rectCorners(center, robotLengthMeters, robotWidthMeters);
    for (Obstacle ob : obstacles) {
      if (ob != null && ob.intersectsRectangle(rect)) return true;
    }
    return false;
  }

  private static boolean segmentCompletelyBlocked(
      Translation2d start,
      Translation2d goal,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    if (obstacles == null || obstacles.isEmpty()) return false;
    int samples = 6;
    for (int i = 0; i <= samples; i++) {
      double s = i / (double) samples;
      double x = start.getX() + (goal.getX() - start.getX()) * s;
      double y = start.getY() + (goal.getY() - start.getY()) * s;
      Translation2d p = new Translation2d(x, y);
      if (!robotIntersects(p, robotLengthMeters, robotWidthMeters, obstacles)) {
        return false;
      }
    }
    return true;
  }

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
      recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
      // Logger.recordOutput(topicRoot + "/Clear", true);
      return true;
    }

    if (segmentCompletelyBlocked(start, goal, robotLengthMeters, robotWidthMeters, obstacles)) {
      recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
      // Logger.recordOutput(topicRoot + "/Clear", false);
      return false;
    }

    final double robotHalfDiag = Math.hypot(robotLengthMeters, robotWidthMeters) * 0.5;
    final double buffer = 0.2;
    final double corridorR = robotHalfDiag + buffer;

    final double GOAL_CAPTURE_RADIUS = 0.20;
    final double PUSH_MARGIN = 0.05;
    final Translation2d goalEff = trimEnd(start, goal, GOAL_CAPTURE_RADIUS);
    recordCircle(topicRoot + "/Goal/Capture", goal, GOAL_CAPTURE_RADIUS, 40);

    final Translation2d seg = goalEff.minus(start);
    final double segLen = Math.max(seg.getNorm(), 1e-9);
    final double terminalT = Math.max(0.0, 1.0 - (GOAL_CAPTURE_RADIUS / segLen));

    recordCorridor(topicRoot + "/Corridor", start, goal, corridorR);
    recordPath(topicRoot + "/Path/Direct", List.of(start, goal));
    if (publishSamples) {
      renderObstacles(topicRoot + "/Obstacles", obstacles, corridorR);
      recordForbiddenGrid(
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
        (obs, penetration) -> isPushableObstacle(obs) && penetration <= PUSH_MARGIN;

    BiFunction<Translation2d, Translation2d, Boolean> segClearAgainstAll =
        (a, b) -> {
          double lminX = Math.min(a.getX(), b.getX()) - corridorR;
          double lmaxX = Math.max(a.getX(), b.getX()) + corridorR;
          double lminY = Math.min(a.getY(), b.getY()) - corridorR;
          double lmaxY = Math.max(a.getY(), b.getY()) + corridorR;

          for (Obstacle obs : obstacles) {
            if (obs == null) continue;

            if (obs instanceof FieldPlanner.PointObstacle p) {
              double eff = p.radius + corridorR;
              if (p.loc.getX() >= lminX
                  && p.loc.getX() <= lmaxX
                  && p.loc.getY() >= lminY
                  && p.loc.getY() <= lmaxY) {
                DistParam dp = pointToSegDistParam(p.loc, a, b);
                if (dp.dist <= eff) {
                  double pen = eff - dp.dist;
                  if (dp.t <= terminalT) return false;
                  if (!allowInWindow.test(obs, pen)) return false;
                }
              }
              continue;
            }

            if (obs instanceof FieldPlanner.SnowmanObstacle s) {
              double eff = s.radius + corridorR;
              if (s.loc.getX() >= lminX
                  && s.loc.getX() <= lmaxX
                  && s.loc.getY() >= lminY
                  && s.loc.getY() <= lmaxY) {
                DistParam dp = pointToSegDistParam(s.loc, a, b);
                if (dp.dist <= eff) {
                  double pen = eff - dp.dist;
                  if (dp.t <= terminalT) return false;
                  if (!allowInWindow.test(obs, pen)) return false;
                }
              }
              continue;
            }

            if (obs instanceof FieldPlanner.HorizontalObstacle h) {
              double t = paramForYOnSegment(h.y, a, b);
              if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
                if (t <= terminalT) return false;
                return false;
              }
              double md = Math.min(Math.abs(a.getY() - h.y), Math.abs(b.getY() - h.y));
              if (md <= corridorR) return false;
              continue;
            }

            if (obs instanceof FieldPlanner.VerticalObstacle v) {
              double t = paramForXOnSegment(v.x, a, b);
              if (!Double.isNaN(t) && t >= 0.0 && t <= 1.0) {
                if (t <= terminalT) return false;
                return false;
              }
              double md = Math.min(Math.abs(a.getX() - v.x), Math.abs(b.getX() - v.x));
              if (md <= corridorR) return false;
              continue;
            }

            if (obs instanceof FieldPlanner.TeardropObstacle tdrop) {
              double eff = tdrop.primaryMaxRange + corridorR;
              DistParam dp = pointToSegDistParam(tdrop.loc, a, b);
              if (dp.dist <= eff) {
                if (dp.t <= terminalT) return false;
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
                DistParam dp = pointToSegDistParam(vo.loc, a, b);
                if (dp.dist <= eff) {
                  double pen = eff - dp.dist;
                  if (dp.t <= terminalT) return false;
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

      if (obs instanceof FieldPlanner.PointObstacle p) {
        double eff = p.radius + corridorR;
        if (inAABB.test(p.loc)) {
          DistParam dp = pointToSegDistParam(p.loc, start, goalEff);
          if (dp.dist <= eff) {
            double pen = eff - dp.dist;
            if (dp.t <= terminalT) {
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

      if (obs instanceof FieldPlanner.SnowmanObstacle s) {
        double eff = s.radius + corridorR;
        if (inAABB.test(s.loc)) {
          DistParam dp = pointToSegDistParam(s.loc, start, goalEff);
          if (dp.dist <= eff) {
            double pen = eff - dp.dist;
            if (dp.t <= terminalT) {
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

      if (obs instanceof FieldPlanner.HorizontalObstacle h) {
        double t = paramForYOnSegment(h.y, start, goalEff);
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

      if (obs instanceof FieldPlanner.VerticalObstacle v) {
        double t = paramForXOnSegment(v.x, start, goalEff);
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

      if (obs instanceof FieldPlanner.TeardropObstacle tdrop) {
        double eff = tdrop.primaryMaxRange + corridorR;
        DistParam dp = pointToSegDistParam(tdrop.loc, start, goalEff);
        if (inAABB.test(tdrop.loc) && dp.dist <= eff) {
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
          DistParam dp = pointToSegDistParam(vo.loc, start, goalEff);
          if (dp.dist <= eff) {
            double pen = eff - dp.dist;
            if (dp.t <= terminalT) {
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
      // Logger.recordOutput(topicRoot + "/Clear", true);
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
      if (obs instanceof FieldPlanner.PointObstacle p) {
        candidates.addAll(offsetAround(p.loc, p.radius + corridorR + 0.05, start, goal));
      } else if (obs instanceof FieldPlanner.SnowmanObstacle s) {
        candidates.addAll(offsetAround(s.loc, s.radius + corridorR + 0.05, start, goal));
      } else if (obs instanceof FieldPlanner.HorizontalObstacle h) {
        double y = h.y + Math.copySign(corridorR + 0.05, goal.getY() - start.getY());
        candidates.add(new Translation2d((start.getX() + goal.getX()) * 0.5, y));
      } else if (obs instanceof FieldPlanner.VerticalObstacle v) {
        double x = v.x + Math.copySign(corridorR + 0.05, goal.getX() - start.getX());
        candidates.add(new Translation2d(x, (start.getY() + goal.getY()) * 0.5));
      } else if (obs instanceof FieldPlanner.TeardropObstacle t) {
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
      recordPoints(topicRoot + "/Candidates", candidates);
    }

    for (Translation2d w : candidates) {
      if (segClearAgainstAll.apply(start, w) && segClearAgainstAll.apply(w, goalEff)) {
        recordPath(topicRoot + "/Path/Chosen", List.of(start, w, goal));
        // Logger.recordOutput(topicRoot + "/Clear", true);
        return true;
      }
    }

    // Logger.recordOutput(topicRoot + "/Clear", false);
    return false;
  }

  private static List<Translation2d> offsetAround(
      Translation2d c, double R, Translation2d start, Translation2d goal) {
    List<Translation2d> out = new ArrayList<>(2);
    Translation2d v = goal.minus(start);
    double vn = v.getNorm();
    if (vn > 1e-9) {
      Translation2d u = new Translation2d(v.getX() / vn, v.getY() / vn);
      Translation2d n = new Translation2d(-u.getY(), u.getX());
      out.add(c.plus(n.times(R)));
      out.add(c.minus(n.times(R)));
    }
    return out;
  }

  private static void recordPath(String key, List<Translation2d> points) {
    // Logger.recordOutput(key, polylineToTrajectory(points));
  }

  private static void recordForbiddenGrid(
      String root,
      List<? extends Obstacle> obstacles,
      double robotLength,
      double robotWidth,
      double step,
      Translation2d goal,
      double goalCaptureRadius) {

    List<Translation2d> blocked = new ArrayList<>();
    List<Translation2d> free = new ArrayList<>();

    for (double x = 0.0; x <= Constants.FIELD_LENGTH; x += step) {
      for (double y = 0.0; y <= Constants.FIELD_WIDTH; y += step) {
        Translation2d c = new Translation2d(x, y);

        if (c.getDistance(goal) <= goalCaptureRadius) {
          free.add(c);
          continue;
        }

        Translation2d[] rect = rectCorners(c, robotLength, robotWidth);
        boolean hit = false;
        for (Obstacle ob : obstacles) {
          if (ob != null && ob.intersectsRectangle(rect)) {
            hit = true;
            break;
          }
        }
        if (hit) blocked.add(c);
        else free.add(c);
      }
    }

    recordPoints(root + "/Points", blocked);
    recordPoints(root + "/Free", free);
  }

  private static void recordCorridor(String root, Translation2d a, Translation2d b, double r) {
    Translation2d d = b.minus(a);
    double n = d.getNorm();
    if (n < 1e-9) return;
    Translation2d u = new Translation2d(d.getX() / n, d.getY() / n);
    Translation2d nvec = new Translation2d(-u.getY(), u.getX());
    Translation2d aL = a.plus(nvec.times(r)), bL = b.plus(nvec.times(r));
    Translation2d aR = a.minus(nvec.times(r)), bR = b.minus(nvec.times(r));
    recordPath(root + "/Left", List.of(aL, bL));
    recordPath(root + "/Right", List.of(aR, bR));
  }

  private static void renderObstacles(
      String root, List<? extends Obstacle> obstacles, double corridorR) {
    int i = 0;
    for (Obstacle ob : obstacles) {
      String k = root + "/#" + (i++);
      if (ob instanceof FieldPlanner.PointObstacle p) {
        recordCircle(k + "/Circle", p.loc, p.radius + corridorR, 40);
      } else if (ob instanceof FieldPlanner.SnowmanObstacle s) {
        recordCircle(k + "/Circle", s.loc, s.radius + corridorR, 40);
      } else if (ob instanceof FieldPlanner.VerticalObstacle v) {
        recordPath(
            k + "/Line",
            List.of(new Translation2d(v.x, 0.0), new Translation2d(v.x, Constants.FIELD_WIDTH)));
      } else if (ob instanceof FieldPlanner.HorizontalObstacle h) {
        recordPath(
            k + "/Line",
            List.of(new Translation2d(0.0, h.y), new Translation2d(Constants.FIELD_LENGTH, h.y)));
      } else if (ob instanceof FieldPlanner.TeardropObstacle t) {
        recordCircle(k + "/Bulb", t.loc, t.primaryMaxRange + corridorR, 40);
        Translation2d tailA = t.loc.plus(new Translation2d(0.0, t.primaryMaxRange + corridorR));
        Translation2d tailB =
            t.loc.plus(new Translation2d(t.tailLength, t.primaryMaxRange + corridorR));
        Translation2d tailC =
            t.loc.plus(new Translation2d(t.tailLength, -t.primaryMaxRange - corridorR));
        Translation2d tailD = t.loc.plus(new Translation2d(0.0, -t.primaryMaxRange - corridorR));
        recordPath(k + "/Tail", List.of(tailA, tailB, tailC, tailD, tailA));
      } else if (ob instanceof VisionObstacle vo) {
        recordEllipse(
            k + "/Ellipse", vo.loc, vo.sizeX * 0.5 + corridorR, vo.sizeY * 0.5 + corridorR, 60);
      }
    }
  }

  private static void recordCircle(String key, Translation2d c, double r, int samples) {
    List<Translation2d> pts = new ArrayList<>(samples + 1);
    for (int i = 0; i <= samples; i++) {
      double th = 2.0 * Math.PI * i / samples;
      pts.add(new Translation2d(c.getX() + r * Math.cos(th), c.getY() + r * Math.sin(th)));
    }
    recordPath(key, pts);
  }

  private static void recordPoints(String key, List<Translation2d> pts) {
    Pose2d[] poses = new Pose2d[pts.size()];
    for (int i = 0; i < pts.size(); i++) {
      poses[i] = new Pose2d(pts.get(i), new Rotation2d());
    }
    // Logger.recordOutput(key, poses);
  }

  private static Trajectory polylineToTrajectory(List<Translation2d> pts) {
    if (pts.size() < 2) {
      Pose2d p = pts.isEmpty() ? new Pose2d() : new Pose2d(pts.get(0), new Rotation2d());
      return new Trajectory(List.of(new Trajectory.State(0.0, 0.0, 0.0, p, 0.0)));
    }
    List<Trajectory.State> states = new ArrayList<>();
    double t = 0.0;
    final double vNom = 1.0;
    for (int i = 0; i < pts.size(); i++) {
      Translation2d p = pts.get(i);
      Translation2d fwd;
      if (i == pts.size() - 1) fwd = p.minus(pts.get(i - 1));
      else fwd = pts.get(i + 1).minus(p);
      double heading = Math.atan2(fwd.getY(), fwd.getX());
      Pose2d pose = new Pose2d(p, new Rotation2d(heading));

      if (i > 0) {
        double ds = p.getDistance(pts.get(i - 1));
        t += ds / vNom;
      }

      double curvature = 0.0;
      if (i > 0 && i < pts.size() - 1) {
        Translation2d a = pts.get(i - 1), b = pts.get(i), c = pts.get(i + 1);
        curvature = estimateCurvature(a, b, c);
      }

      states.add(new Trajectory.State(t, vNom, 0.0, pose, curvature));
    }
    return new Trajectory(states);
  }

  private static double estimateCurvature(Translation2d a, Translation2d b, Translation2d c) {
    double x1 = a.getX(), y1 = a.getY();
    double x2 = b.getX(), y2 = b.getY();
    double x3 = c.getX(), y3 = c.getY();
    double d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (Math.abs(d) < 1e-9) return 0.0;
    double ux =
        ((x1 * x1 + y1 * y1) * (y2 - y3)
                + (x2 * x2 + y2 * y2) * (y3 - y1)
                + (x3 * x3 + y3 * y3) * (y1 - y2))
            / d;
    double uy =
        ((x1 * x1 + y1 * y1) * (x3 - x2)
                + (x2 * x2 + y2 * y2) * (x1 - x3)
                + (x3 * x3 + y3 * y3) * (x2 - x1))
            / d;
    double R = Math.hypot(b.getX() - ux, b.getY() - uy);
    return (R < 1e-6) ? 0.0 : 1.0 / R;
  }

  public class BounceListener {
    private final double bounceDistanceThreshold;
    private final int bounceHistoryLimit;
    private final Queue<Pose2d> recentGoals = new LinkedList<>();
    private boolean isBouncing;

    public BounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
      this.bounceDistanceThreshold = bounceDistanceThreshold;
      this.bounceHistoryLimit = bounceHistoryLimit;
    }

    public void update(Pose2d currentGoal) {
      recentGoals.add(currentGoal);
      if (recentGoals.size() > bounceHistoryLimit) {
        recentGoals.poll();
      }

      isBouncing = checkBouncing();
    }

    private boolean checkBouncing() {
      if (recentGoals.size() < bounceHistoryLimit) return false;

      int similarCount = 0;
      Pose2d[] goals = recentGoals.toArray(new Pose2d[0]);
      for (int i = 0; i < goals.length - 1; i++) {
        for (int j = i + 1; j < goals.length; j++) {
          if (goals[i].getTranslation().getDistance(goals[j].getTranslation())
              < bounceDistanceThreshold) {
            similarCount++;
          }
        }
      }

      int totalPairs = (goals.length * (goals.length - 1)) / 2;
      return similarCount >= (totalPairs * 0.6);
    }

    public void clearHistory() {
      recentGoals.clear();
      isBouncing = false;
    }

    public boolean isBouncing() {
      return isBouncing;
    }
  }
}
