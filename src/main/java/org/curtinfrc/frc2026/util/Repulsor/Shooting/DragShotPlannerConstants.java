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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

final class DragShotPlannerConstants {
  static final double EPS = 1e-6;
  static final double MIN_RANGE_METERS = 0.5;
  static final double MAX_RANGE_METERS = 7.0;
  static final double MAX_ROBOT_TRAVEL_METERS = 7.0;
  static final double MAX_ROBOT_TRAVEL_METERS_SQ =
      MAX_ROBOT_TRAVEL_METERS * MAX_ROBOT_TRAVEL_METERS;
  static final double ACCEPTABLE_VERTICAL_ERROR_METERS = 0.06;
  static final double FAST_ACCEPTABLE_VERTICAL_ERROR_METERS = 0.25;
  static final double DEG_TO_RAD = Math.PI / 180.0;
  static final double RAD_TO_DEG = 180.0 / Math.PI;

  private DragShotPlannerConstants() {}
}
