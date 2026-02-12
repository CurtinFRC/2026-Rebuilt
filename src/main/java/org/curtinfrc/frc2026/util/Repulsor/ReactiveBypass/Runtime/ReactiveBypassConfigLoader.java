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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

final class ReactiveBypassConfigLoader {
  private ReactiveBypassConfigLoader() {}

  static void loadConfigFromYaml(Object cfg, Class<?> schemaClass) {
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      Path yamlPath = deployDir.resolve("ReactiveBypassConfig.yaml");
      if (!Files.exists(yamlPath)) return;
      List<String> lines = Files.readAllLines(yamlPath, StandardCharsets.UTF_8);
      for (String raw : lines) {
        String line = raw.trim();
        if (line.isEmpty()) continue;
        if (line.startsWith("#")) continue;
        int idx = line.indexOf(':');
        if (idx <= 0) continue;
        String key = line.substring(0, idx).trim();
        String valueStr = line.substring(idx + 1).trim();
        if (!valueStr.isEmpty()) applyConfigField(cfg, schemaClass, key, valueStr);
      }
    } catch (IOException e) {
      System.err.println("ReactiveBypass: Failed to load config from YAML: " + e);
    }
  }

  private static void applyConfigField(
      Object cfg, Class<?> schemaClass, String key, String valueStr) {
    try {
      Field f = schemaClass.getField(key);
      Class<?> t = f.getType();
      if (t == double.class) {
        double v = Double.parseDouble(valueStr);
        f.setDouble(cfg, v);
      } else if (t == int.class) {
        int v = Integer.parseInt(valueStr);
        f.setInt(cfg, v);
      } else if (t == boolean.class) {
        boolean v = Boolean.parseBoolean(valueStr);
        f.setBoolean(cfg, v);
      }
    } catch (NoSuchFieldException | IllegalAccessException | IllegalArgumentException ignored) {
    }
  }
}
