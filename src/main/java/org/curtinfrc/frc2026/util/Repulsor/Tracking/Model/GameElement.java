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

import java.util.Optional;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints.RepulsorSetpoint;

public class GameElement {
  private final GameObject[] containedStorage;
  private int containedCount;
  private int maxContained;

  private GameElementModel model;
  private Predicate<GameObject> filter;
  private Optional<RepulsorSetpoint> relatedPoint = Optional.empty();
  private CategorySpec category;
  private Alliance alliance;

  private GameObject[] cachedExact;
  private boolean dirtyCache;

  public GameElement(GameElementModel model) {
    this(Alliance.kBlue, 10, model, g -> true, null, CategorySpec.kScore);
  }

  public GameElement(
      Alliance alliance,
      int maxContained,
      GameElementModel model,
      Predicate<GameObject> filter,
      RepulsorSetpoint relatedPoint,
      CategorySpec category) {
    if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
    if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
    this.alliance = alliance;
    this.maxContained = maxContained;
    this.model = model;
    this.filter = (filter != null) ? filter : (go -> true);
    if (relatedPoint != null) this.relatedPoint = Optional.ofNullable(relatedPoint);
    this.category = category;
    this.containedStorage = new GameObject[Math.max(1, maxContained)];
    this.containedCount = 0;
    this.cachedExact = new GameObject[0];
    this.dirtyCache = true;
  }

  public boolean filter(GameObject gameObject) {
    if (gameObject == null) throw new IllegalArgumentException("GameObject cannot be null");
    return filter.test(gameObject);
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public CategorySpec getCategory() {
    return category;
  }

  public void setAlliance(Alliance alliance) {
    if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
    this.alliance = alliance;
  }

  public int getMaxContained() {
    return maxContained;
  }

  public void setMaxContained(int maxContained) {
    if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
    this.maxContained = maxContained;
    if (containedCount > maxContained) {
      containedCount = maxContained;
      dirtyCache = true;
    }
  }

  public GameObject[] getContained() {
    if (!dirtyCache && cachedExact.length == containedCount) {
      return cachedExact;
    }
    GameObject[] out = new GameObject[containedCount];
    if (containedCount > 0) {
      System.arraycopy(containedStorage, 0, out, 0, containedCount);
    }
    cachedExact = out;
    dirtyCache = false;
    return out;
  }

  public void setContained(GameObject[] contained) {
    clearContained();
    if (contained == null || contained.length == 0) return;
    int n = Math.min(contained.length, maxContained);
    for (int i = 0; i < n; i++) {
      containedStorage[i] = contained[i];
    }
    containedCount = n;
    dirtyCache = true;
  }

  public GameObject getContained(int index) {
    if (index < 0 || index >= containedCount) {
      throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + containedCount);
    }
    return containedStorage[index];
  }

  public int getContainedCount() {
    return containedCount;
  }

  public boolean isAtCapacity() {
    return containedCount >= maxContained;
  }

  public GameElementModel getModel() {
    return model;
  }

  public void setModel(GameElementModel model) {
    this.model = model;
  }

  public Optional<RepulsorSetpoint> getRelatedPoint() {
    return relatedPoint;
  }

  public void setRelatedPoint(RepulsorSetpoint newPoint) {
    this.relatedPoint = Optional.ofNullable(newPoint);
  }

  public void setFilter(Predicate<GameObject> filter) {
    this.filter = (filter != null) ? filter : (go -> true);
  }

  public void setCategory(CategorySpec category) {
    this.category = category;
  }

  public void clearContained() {
    if (containedCount != 0) {
      for (int i = 0; i < containedCount; i++) {
        containedStorage[i] = null;
      }
      containedCount = 0;
      dirtyCache = true;
    }
  }

  public boolean tryAdd(GameObject obj) {
    if (obj == null) return false;
    if (containedCount >= maxContained) return false;
    containedStorage[containedCount++] = obj;
    dirtyCache = true;
    return true;
  }
}
