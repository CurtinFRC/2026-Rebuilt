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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model;

import edu.wpi.first.math.geometry.Pose3d;

public final class GameObject {
  private final String id;
  private final String type;
  private final Pose3d position;

  public GameObject(String id, String type) {
    this(id, type, null);
  }

  public GameObject(String id, String type, Pose3d position) {
    if (id == null || id.isEmpty()) throw new IllegalArgumentException("id cannot be null/empty");
    if (type == null || type.isEmpty())
      throw new IllegalArgumentException("type cannot be null/empty");
    this.id = id;
    this.type = type;
    this.position = position;
  }

  public String getId() {
    return id;
  }

  public String getType() {
    return type;
  }

  public Pose3d getPosition() {
    return position;
  }
}
