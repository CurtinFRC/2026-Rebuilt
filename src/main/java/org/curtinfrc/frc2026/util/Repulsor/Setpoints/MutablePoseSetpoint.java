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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.concurrent.atomic.AtomicReference;

public final class MutablePoseSetpoint extends GameSetpoint {
  private final AtomicReference<Pose2d> bluePoseRef;

  public MutablePoseSetpoint(String name, SetpointType type, AtomicReference<Pose2d> bluePoseRef) {
    super(name, type == null ? SetpointType.kOther : type, false);
    this.bluePoseRef = bluePoseRef == null ? new AtomicReference<>(Pose2d.kZero) : bluePoseRef;
  }

  @Override
  public Pose2d bluePose(SetpointContext ctx) {
    Pose2d p = bluePoseRef.get();
    return p != null ? p : Pose2d.kZero;
  }
}
