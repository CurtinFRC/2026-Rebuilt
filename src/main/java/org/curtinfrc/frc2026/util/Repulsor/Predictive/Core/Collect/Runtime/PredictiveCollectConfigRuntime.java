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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;

public final class PredictiveCollectConfigRuntime {
  private PredictiveCollectConfigRuntime() {}

  public static void registerResourceSpec(
      PredictiveFieldStateOps ops, String type, ResourceSpec spec) {
    if (type == null || type.isEmpty() || spec == null) return;
    ops.resourceSpecs.put(type.toLowerCase(), spec);
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  public static void registerOtherTypeWeight(
      PredictiveFieldStateOps ops, String type, double weight) {
    if (type == null || type.isEmpty()) return;
    ops.otherTypeWeights.put(type.toLowerCase(), Math.max(0.0, weight));
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  public static void setCollectResourceTypes(PredictiveFieldStateOps ops, Set<String> types) {
    ops.collectResourceTypes.clear();
    if (types != null) {
      for (String type : types) {
        if (type == null || type.isEmpty()) continue;
        ops.collectResourceTypes.add(type.toLowerCase());
      }
    }
    if (ops.collectResourceTypes.isEmpty()) {
      ops.collectResourceTypes.add(PredictiveFieldStateOps.DEFAULT_COLLECT_RESOURCE_TYPE);
    }
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  public static Set<String> getCollectResourceTypes(PredictiveFieldStateOps ops) {
    return Collections.unmodifiableSet(new HashSet<>(ops.collectResourceTypes));
  }

  public static void addCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return;
    if (ops.collectResourceTypes.add(type.toLowerCase())) {
      ops.specsVersion++;
      ops.invalidateDynCache();
    }
  }

  public static void removeCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return;
    if (ops.collectResourceTypes.remove(type.toLowerCase())) {
      if (ops.collectResourceTypes.isEmpty()) {
        ops.collectResourceTypes.add(PredictiveFieldStateOps.DEFAULT_COLLECT_RESOURCE_TYPE);
      }
      ops.specsVersion++;
      ops.invalidateDynCache();
    }
  }

  public static boolean isCollectResourceType(PredictiveFieldStateOps ops, String type) {
    if (type == null || type.isEmpty()) return false;
    return ops.collectResourceTypes.contains(type.toLowerCase());
  }

  public static void setCollectResourcePositionFilter(
      PredictiveFieldStateOps ops, Predicate<Translation2d> filter) {
    ops.collectResourcePositionFilter =
        filter != null ? filter : PredictiveFieldStateOps::defaultCollectResourcePositionFilter;
    ops.specsVersion++;
    ops.invalidateDynCache();
  }

  public static void setDynamicObjects(PredictiveFieldStateOps ops, List<DynamicObject> objs) {
    ops.dynamicObjects = (objs != null) ? List.copyOf(objs) : List.of();
    ops.invalidateDynCache();
  }

  public static void setWorld(
      PredictiveFieldStateOps ops, List<GameElement> elements, Alliance ours) {
    ops.worldElements = elements != null ? elements : List.of();
    ops.ourAlliance = ours;
  }
}
