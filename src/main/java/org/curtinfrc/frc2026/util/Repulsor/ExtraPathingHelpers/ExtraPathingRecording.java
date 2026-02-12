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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.SnowmanObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.TeardropObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner.VisionObstacle;

final class ExtraPathingRecording {
  private ExtraPathingRecording() {}

  static void recordEllipse(String key, Translation2d c, double rx, double ry, int samples) {
    List<Translation2d> pts = new ArrayList<>(samples + 1);
    for (int i = 0; i <= samples; i++) {
      double th = 2.0 * Math.PI * i / samples;
      pts.add(new Translation2d(c.getX() + rx * Math.cos(th), c.getY() + ry * Math.sin(th)));
    }
    recordPath(key, pts);
  }

  static void recordPath(String key, List<Translation2d> points) {
    // Logger.recordOutput(key, polylineToTrajectory(points));
  }

  static void recordForbiddenGrid(
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

        Translation2d[] rect = ExtraPathingCollision.rectCorners(c, robotLength, robotWidth);
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

  static void recordCorridor(String root, Translation2d a, Translation2d b, double r) {
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

  static void renderObstacles(String root, List<? extends Obstacle> obstacles, double corridorR) {
    int i = 0;
    for (Obstacle ob : obstacles) {
      String k = root + "/#" + (i++);
      if (ob instanceof PointObstacle p) {
        recordCircle(k + "/Circle", p.loc, p.radius + corridorR, 40);
      } else if (ob instanceof SnowmanObstacle s) {
        recordCircle(k + "/Circle", s.loc, s.radius + corridorR, 40);
      } else if (ob instanceof VerticalObstacle v) {
        recordPath(
            k + "/Line",
            List.of(new Translation2d(v.x, 0.0), new Translation2d(v.x, Constants.FIELD_WIDTH)));
      } else if (ob instanceof HorizontalObstacle h) {
        recordPath(
            k + "/Line",
            List.of(new Translation2d(0.0, h.y), new Translation2d(Constants.FIELD_LENGTH, h.y)));
      } else if (ob instanceof TeardropObstacle t) {
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

  static void recordCircle(String key, Translation2d c, double r, int samples) {
    List<Translation2d> pts = new ArrayList<>(samples + 1);
    for (int i = 0; i <= samples; i++) {
      double th = 2.0 * Math.PI * i / samples;
      pts.add(new Translation2d(c.getX() + r * Math.cos(th), c.getY() + r * Math.sin(th)));
    }
    recordPath(key, pts);
  }

  static void recordPoints(String key, List<Translation2d> pts) {
    Pose2d[] poses = new Pose2d[pts.size()];
    for (int i = 0; i < pts.size(); i++) {
      poses[i] = new Pose2d(pts.get(i), new Rotation2d());
    }
    // Logger.recordOutput(key, poses);
  }

  static Trajectory polylineToTrajectory(List<Translation2d> pts) {
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
        curvature = ExtraPathingMath.estimateCurvature(a, b, c);
      }

      states.add(new Trajectory.State(t, vNom, 0.0, pose, curvature));
    }
    return new Trajectory(states);
  }
}
