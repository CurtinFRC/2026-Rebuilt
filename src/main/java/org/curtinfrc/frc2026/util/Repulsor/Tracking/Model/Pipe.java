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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Model;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Pipe extends PrimitiveObject {
  private Pose3d position;
  private Distance radius;
  private Angle angle;

  public Pipe(Pose3d position, Distance radius) {
    this(position, radius, Radians.of(0));
  }

  public Pipe(Pose3d position, Distance radius, Angle angle) {
    this.position = position;
    this.radius = radius;
    this.angle = angle;
  }

  public Pose3d getPosition() {
    return position;
  }

  public void setPosition(Pose3d position) {
    this.position = position;
  }

  public Distance getRadius() {
    return radius;
  }

  public void setRadius(Distance radius) {
    this.radius = radius;
  }

  public Angle getAngle() {
    return angle;
  }

  public void setAngle(Angle angle) {
    this.angle = angle;
  }

  @Override
  public boolean intersects(Pose3d pos) {
    double dx = pos.getX() - position.getX();
    double dy = pos.getY() - position.getY();
    double dz = pos.getZ() - position.getZ();
    double r = radius.in(Meters);
    return (dx * dx + dy * dy + dz * dz) <= r * r;
  }
}
