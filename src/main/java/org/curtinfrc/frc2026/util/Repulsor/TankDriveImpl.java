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

package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public abstract class TankDriveImpl extends DriveRepulsor {
  private double lastLeftMS = 0.0;
  private double lastRightMS = 0.0;
  private double lastCmdTime = Timer.getFPGATimestamp();
  private double lastLeftVolts = 0.0;
  private double lastRightVolts = 0.0;

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    double dt = Math.max(1e-3, Timer.getFPGATimestamp() - lastCmdTime);
    lastCmdTime += dt;

    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double omegaPID = speeds.omegaRadiansPerSecond;

    double theta = Math.atan2(vy, Math.max(1e-9, vx));
    double omegaArc = getArcAlignGain() * theta;
    double omegaEff = omegaPID + omegaArc;

    double vFwd = vx;
    vFwd = applyDeadband(vFwd, getLinearDeadband());
    omegaEff = applyDeadband(omegaEff, getAngularDeadband());

    double track = Math.max(1e-6, getTrackWidth());
    double leftMS;
    double rightMS;

    if (Math.abs(vFwd) < getInPlaceTurnVXThreshold()) {
      leftMS = -omegaEff * 0.5 * track;
      rightMS = +omegaEff * 0.5 * track;
    } else {
      double curvature = clamp(omegaEff / vFwd, -getMaxCurvature(), getMaxCurvature());
      leftMS = vFwd * (1.0 - curvature * 0.5 * track);
      rightMS = vFwd * (1.0 + curvature * 0.5 * track);
    }

    double maxLin = Math.max(1e-6, getMaxLinearMetersPerSec());
    double lScale = Math.abs(leftMS) / maxLin;
    double rScale = Math.abs(rightMS) / maxLin;
    double scale = Math.max(1.0, Math.max(lScale, rScale));
    leftMS /= scale;
    rightMS /= scale;

    double tgtLeft = slewLinear(leftMS, lastLeftMS, dt);
    double tgtRight = slewLinear(rightMS, lastRightMS, dt);

    if (Math.abs(vFwd) < getInPlaceTurnVXThreshold()) {
      double omegaRate = Math.abs(omegaEff);
      double maxOmegaRate = getMaxOmegaSlew();
      double omegaSlew = clamp(omegaRate, 0.0, maxOmegaRate);
      double sign = Math.signum(omegaEff);
      double turnMS = omegaSlew * 0.5 * track;
      tgtLeft = -sign * turnMS;
      tgtRight = +sign * turnMS;
    } else {
      double curvSlew = getCurvatureSlew();
      double dLeft = tgtLeft - lastLeftMS;
      double dRight = tgtRight - lastRightMS;
      double curvFactor = clamp(1.0 - curvSlew * dt, 0.0, 1.0);
      double sym = 0.5 * (dLeft + dRight);
      double anti = 0.5 * (dRight - dLeft) * curvFactor;
      double dL2 = sym - anti;
      double dR2 = sym + anti;
      tgtLeft = lastLeftMS + dL2;
      tgtRight = lastRightMS + dR2;
    }

    double leftAcc = (tgtLeft - lastLeftMS) / dt;
    double rightAcc = (tgtRight - lastRightMS) / dt;
    lastLeftMS = tgtLeft;
    lastRightMS = tgtRight;

    double ks = getKs();
    double kv = getKv();
    double ka = getKa();
    double maxVolt = Math.max(1.0, getMaxVoltage());

    double leftVff = ks * Math.signum(tgtLeft) + kv * tgtLeft + ka * leftAcc;
    double rightVff = ks * Math.signum(tgtRight) + kv * tgtRight + ka * rightAcc;

    double maxStepV = getMaxVoltPerSec() * dt;
    double leftV = slewVolts(clamp(leftVff, -maxVolt, +maxVolt), lastLeftVolts, maxStepV);
    double rightV = slewVolts(clamp(rightVff, -maxVolt, +maxVolt), lastRightVolts, maxStepV);
    lastLeftVolts = leftV;
    lastRightVolts = rightV;

    if (Math.abs(leftV) < getNeutralDeadbandVolts()
        && Math.abs(rightV) < getNeutralDeadbandVolts()) {
      leftV = 0.0;
      rightV = 0.0;
    }

    drive(Volts.of(leftV), Volts.of(rightV));
  }

  protected abstract void drive(Voltage left, Voltage right);

  protected abstract double getTrackWidth();

  protected abstract double getMaxLinearMetersPerSec();

  protected abstract double getMaxVoltage();

  protected double getArcAlignGain() {
    return 2.0;
  }

  protected double getLinearSlew() {
    return 4.0;
  }

  protected double getAngularDeadband() {
    return 0.02;
  }

  protected double getLinearDeadband() {
    return 0.01;
  }

  protected double getMaxCurvature() {
    return 2.5;
  }

  protected double getCurvatureSlew() {
    return 6.0;
  }

  protected double getInPlaceTurnVXThreshold() {
    return 0.15;
  }

  protected double getMaxOmegaSlew() {
    return 4.0;
  }

  protected double getMaxVoltPerSec() {
    return 64.0;
  }

  protected double getNeutralDeadbandVolts() {
    return 0.2;
  }

  protected double getKs() {
    return 0.6;
  }

  protected double getKv() {
    return getMaxVoltage() / Math.max(0.01, getMaxLinearMetersPerSec());
  }

  protected double getKa() {
    return 0.0;
  }

  private static double applyDeadband(double v, double db) {
    return Math.abs(v) < db ? 0.0 : v;
  }

  private static double clamp(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  private double slewLinear(double tgt, double prev, double dt) {
    double maxStep = getLinearSlew() * dt;
    double d = tgt - prev;
    if (d > maxStep) return prev + maxStep;
    if (d < -maxStep) return prev - maxStep;
    return tgt;
  }

  private static double slewVolts(double tgt, double prev, double maxStep) {
    double d = tgt - prev;
    if (d > maxStep) return prev + maxStep;
    if (d < -maxStep) return prev - maxStep;
    return tgt;
  }
}
