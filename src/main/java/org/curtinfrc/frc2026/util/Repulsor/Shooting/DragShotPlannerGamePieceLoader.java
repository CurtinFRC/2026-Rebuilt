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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.ConcurrentHashMap;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

final class DragShotPlannerGamePieceLoader {
  private static final ConcurrentHashMap<String, GamePiecePhysics> GAME_PIECE_CACHE =
      new ConcurrentHashMap<>();

  private DragShotPlannerGamePieceLoader() {}

  static GamePiecePhysics loadGamePieceFromDeployYaml(String id) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.loadGamePieceFromDeployYaml");
    try {
      if (id == null || id.isEmpty()) {
        throw new IllegalArgumentException("id must be non-empty");
      }
      return GAME_PIECE_CACHE.computeIfAbsent(
          id, DragShotPlannerGamePieceLoader::loadGamePieceFromDeployYamlInternal);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static GamePiecePhysics loadGamePieceFromDeployYamlInternal(String id) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.loadGamePieceFromDeployYamlInternal");
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      Path gamePiecesDir = deployDir.resolve("gamepieces");
      String fileName = id.endsWith(".yml") || id.endsWith(".yaml") ? id : id + ".yaml";
      Path path = gamePiecesDir.resolve(fileName);
      if (!Files.exists(path)) {
        DriverStation.reportError("Game piece YAML not found: " + path.toString(), false);
        throw new IllegalStateException("Missing game piece YAML: " + path.toString());
      }
      Yaml yaml = new Yaml(new Constructor(GamePiecePhysicsConfig.class, new LoaderOptions()));
      try (InputStream in = Files.newInputStream(path)) {
        GamePiecePhysicsConfig cfg = yaml.load(in);
        if (cfg == null) {
          DriverStation.reportError("Game piece YAML empty: " + path.toString(), false);
          throw new IllegalStateException("Game piece YAML empty: " + path.toString());
        }
        double mass =
            (cfg.metadata != null
                    && cfg.metadata.mass_kg_override != null
                    && cfg.metadata.mass_kg_override > 0.0)
                ? cfg.metadata.mass_kg_override
                : cfg.mass_kg;
        double area = cfg.cross_section_area_m2;
        double cd = cfg.drag_coefficient;
        double air =
            cfg.air_density_kg_per_m3 > 0.0
                ? cfg.air_density_kg_per_m3
                : 1.225;

        if (mass <= 0.0 || area <= 0.0 || cd <= 0.0 || air <= 0.0) {
          DriverStation.reportError("Invalid game piece YAML values: " + path.toString(), false);
          throw new IllegalStateException("Invalid game piece YAML: " + path.toString());
        }
        String name = cfg.name != null && !cfg.name.isEmpty() ? cfg.name : id;
        return new YamlGamePiecePhysics(name, mass, area, cd, air);
      } catch (IOException ex) {
        DriverStation.reportError("Failed to read game piece YAML: " + path.toString() + " - " + ex.getMessage(), false);
        throw new IllegalStateException("Failed to read game piece YAML: " + path.toString() + " - " + ex.getMessage(), ex);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Failed to parse game piece YAML: " + path.toString() + " - " + ex.getMessage(), true);
        throw ex;
      }
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  private static final class YamlGamePiecePhysics extends GamePiecePhysics {
    private final String name;
    private final double massKg;
    private final double crossSectionAreaM2;
    private final double dragCoefficient;
    private final double airDensityKgPerM3;

    YamlGamePiecePhysics(
        String name,
        double massKg,
        double crossSectionAreaM2,
        double dragCoefficient,
        double airDensityKgPerM3) {
      this.name = name;
      this.massKg = massKg;
      this.crossSectionAreaM2 = crossSectionAreaM2;
      this.dragCoefficient = dragCoefficient;
      this.airDensityKgPerM3 = airDensityKgPerM3;
    }

    @SuppressWarnings("unused")
    public String name() {
      return name;
    }

    @Override
    public double massKg() {
      return massKg;
    }

    @Override
    public double crossSectionAreaM2() {
      return crossSectionAreaM2;
    }

    @Override
    public double dragCoefficient() {
      return dragCoefficient;
    }

    @Override
    public double airDensityKgPerM3() {
      return airDensityKgPerM3;
    }
  }
}
