/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Vision.Test;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Random;

public final class VisionSimWorld {
  public static final double FIELD_LENGTH = 16.0;
  public static final double FIELD_WIDTH = 8.0;
  public static final double MAX_SPEED = 4.3;
  public static final double MAX_ACCEL = 4.2;
  public static final double OU_TAU_MEAN = 1.1;
  public static final double OU_SIGMA_MEAN = 1.7;
  public static final double HEADING_LERP = 0.22;
  public static final double WALL_MARGIN = 0.55;
  public static final double WALL_GAIN = 2.6;
  public static final double DT_MIN = 0.005;
  public static final double DT_MAX = 0.04;
  private static volatile Pose2d selfPose = null;

  public static void setSelfPose(Pose2d p) {
    selfPose = p;
  }

  public static Pose2d getSelfPose() {
    return selfPose;
  }

  public static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  public static double wrapPi(double a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  public static double lerpAngle(double a, double b, double t) {
    double d = wrapPi(b - a);
    return a + t * d;
  }

  public static class OU {
    private final Random rng;
    private final double tau;
    private final double sigma;
    private double ax;
    private double ay;

    public OU(Random rng, double tau, double sigma) {
      this.rng = rng;
      this.tau = tau;
      this.sigma = sigma;
    }

    public double[] step(double dt) {
      double k = Math.exp(-dt / tau);
      double s = sigma * Math.sqrt(1 - k * k);
      ax = k * ax + s * rng.nextGaussian();
      ay = k * ay + s * rng.nextGaussian();
      return new double[] {ax, ay};
    }
  }

  public static class Agent {
    public double x, y, vx, vy, heading;
    public final OU ou;
    public final AgentRole role;
    public int team;
    public double rx, ry;

    public double maxSpeed;
    public double maxAccel;

    public Agent(Random rng, AgentRole role, int team, double rx, double ry) {
      this.role = role;
      this.team = team;
      this.rx = rx;
      this.ry = ry;
      this.x = 0.8 + rng.nextDouble() * (FIELD_LENGTH - 1.6);
      this.y = 0.6 + rng.nextDouble() * (FIELD_WIDTH - 1.2);
      this.vx = (rng.nextDouble() - 0.5) * 0.8;
      this.vy = (rng.nextDouble() - 0.5) * 0.8;
      this.heading = Math.atan2(vy, vx);
      double tau = OU_TAU_MEAN * (0.75 + 0.5 * rng.nextDouble());
      double sig = OU_SIGMA_MEAN * (0.75 + 0.6 * rng.nextDouble());
      this.ou = new OU(rng, tau, sig);
      this.maxSpeed = MAX_SPEED;
      this.maxAccel = MAX_ACCEL;
    }

    public Agent withCaps(double speed, double accel) {
      this.maxSpeed = speed;
      this.maxAccel = accel;
      return this;
    }
  }
}
