package org.curtinfrc.frc2026.util.Repulsor.Offload.Server;

import java.io.IOException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.ServiceLoader;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction;

final class OffloadPluginLoader {
  private OffloadPluginLoader() {}

  static LoadedPlugins load(Path pluginDirectory) throws IOException {
    Map<String, OffloadFunction<?, ?>> functionsById = new HashMap<>();
    List<ClassLoader> classLoaders = new ArrayList<>();

    if (!Files.exists(pluginDirectory)) {
      Files.createDirectories(pluginDirectory);
    }

    try (var paths = Files.list(pluginDirectory)) {
      paths
          .filter(path -> path.toString().endsWith(".jar"))
          .forEach(
              jarPath -> {
                try {
                  URL jarUrl = jarPath.toUri().toURL();
                  URLClassLoader classLoader =
                      new URLClassLoader(
                          new URL[] {jarUrl}, OffloadPluginLoader.class.getClassLoader());
                  classLoaders.add(classLoader);

                  ServiceLoader<OffloadFunction> serviceLoader =
                      ServiceLoader.load(OffloadFunction.class, classLoader);
                  for (OffloadFunction<?, ?> function : serviceLoader) {
                    functionsById.put(function.taskId(), function);
                  }
                } catch (Exception ex) {
                  throw new RuntimeException("Failed to load plugin JAR " + jarPath, ex);
                }
              });
    }

    return new LoadedPlugins(functionsById, classLoaders);
  }

  record LoadedPlugins(
      Map<String, OffloadFunction<?, ?>> functionsById, List<ClassLoader> classLoaders) {}
}
