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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameObject;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.PrimitiveObject;

public class FieldVision {
  private static final int MAX_OBJECTS_PER_TICK = 256;

  private static final class FieldVisionData {
    private final Pose3d position;
    private final String type;

    FieldVisionData(Pose3d position, String type) {
      this.position = position;
      this.type = type;
    }

    public Pose3d getPosition() {
      return position;
    }

    public String getType() {
      return type;
    }
  }

  private final FieldTrackerCore owner;
  private final String name;
  private final String host;
  private final NetworkTable table;

  private final HashMap<String, FieldVisionData> objects = new HashMap<>(256);

  public FieldVision(FieldTrackerCore owner, String name) {
    if (owner == null) throw new IllegalArgumentException("owner cannot be null");
    if (name == null || name.isEmpty())
      throw new IllegalArgumentException("name cannot be null/empty");
    this.owner = owner;
    this.name = name;
    this.host = name + "-vision";
    this.table = NetworkTableInstance.getDefault().getTable("FieldVision/" + name);
  }

  public String getName() {
    return name;
  }

  public String getHost() {
    return host;
  }

  public void update(Pose2d currentPose) {
    if (currentPose == null) return;

    Pose3d field_T_robot =
        new Pose3d(
            currentPose.getX(),
            currentPose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));

    double ex = table.getEntry("extrinsics/x").getDouble(0.0);
    double ey = table.getEntry("extrinsics/y").getDouble(0.0);
    double ez = table.getEntry("extrinsics/z").getDouble(0.0);
    double eroll = table.getEntry("extrinsics/roll").getDouble(0.0);
    double epitch = table.getEntry("extrinsics/pitch").getDouble(0.0);
    double eyaw = table.getEntry("extrinsics/yaw").getDouble(0.0);

    Transform3d robot_T_camera =
        new Transform3d(new Translation3d(ex, ey, ez), new Rotation3d(eroll, epitch, eyaw));

    Pose3d field_T_camera =
        field_T_robot.transformBy(
            new Transform3d(robot_T_camera.getTranslation(), robot_T_camera.getRotation()));

    objects.clear();
    Set<String> keys = table.getKeys();
    int seen = 0;

    long nowNs = System.nanoTime();

    for (String key : keys) {
      if (seen >= MAX_OBJECTS_PER_TICK) break;
      if (!key.startsWith("object_")) continue;

      String objectId = key.substring(7);

      String frame = table.getEntry(key + "/frame").getString("field");

      double roll = table.getEntry(key + "/roll").getDouble(0.0);
      double pitch = table.getEntry(key + "/pitch").getDouble(0.0);
      double yaw = table.getEntry(key + "/yaw").getDouble(0.0);
      Rotation3d localRot = new Rotation3d(roll, pitch, yaw);

      String rawType = table.getEntry(key + "/type").getString("unknown");
      String type = owner.canonicalizeType(rawType);

      Pose3d fieldPose;

      if ("camera".equalsIgnoreCase(frame)) {
        double px = table.getEntry(key + "/px").getDouble(0.0);
        double py = table.getEntry(key + "/py").getDouble(0.0);
        double pz = table.getEntry(key + "/pz").getDouble(0.0);
        Pose3d camera_T_object = new Pose3d(px, py, pz, localRot);
        fieldPose =
            field_T_camera.transformBy(
                new Transform3d(camera_T_object.getTranslation(), camera_T_object.getRotation()));
      } else if ("robot".equalsIgnoreCase(frame)) {
        double px = table.getEntry(key + "/px").getDouble(0.0);
        double py = table.getEntry(key + "/py").getDouble(0.0);
        double pz = table.getEntry(key + "/pz").getDouble(0.0);
        Pose3d robot_T_object = new Pose3d(px, py, pz, localRot);
        fieldPose =
            field_T_robot.transformBy(
                new Transform3d(robot_T_object.getTranslation(), robot_T_object.getRotation()));
      } else {
        double x = table.getEntry(key + "/x").getDouble(0.0);
        double y2 = table.getEntry(key + "/y").getDouble(0.0);
        double z = table.getEntry(key + "/z").getDouble(0.0);
        fieldPose = new Pose3d(x, y2, z, localRot);
      }

      objects.put(objectId, new FieldVisionData(fieldPose, type));
      owner.ingestTracked(objectId, type, fieldPose, nowNs);
      seen++;
    }

    GameElement[] fm = owner.field_map;
    if (fm == null || fm.length == 0) return;

    for (GameElement element : fm) {
      if (element != null) element.clearContained();
    }

    for (var entry : objects.entrySet()) {
      String objectId = entry.getKey();
      FieldVisionData data = entry.getValue();
      Pose3d position = data.getPosition();
      String type = data.getType();
      GameObject obj = new GameObject(objectId, type, position);

      for (GameElement element : fm) {
        if (element == null) continue;
        if (element.getContainedCount() >= element.getMaxContained()) continue;
        if (!element.filter(obj)) continue;

        PrimitiveObject[] primitives =
            element.getModel() != null ? element.getModel().getComposition() : null;
        if (primitives == null || primitives.length == 0) continue;

        boolean hit = false;
        for (PrimitiveObject primitive : primitives) {
          if (primitive != null && primitive.intersects(position)) {
            hit = true;
            break;
          }
        }
        if (hit) {
          element.tryAdd(obj);
        }
      }
    }
  }
}
