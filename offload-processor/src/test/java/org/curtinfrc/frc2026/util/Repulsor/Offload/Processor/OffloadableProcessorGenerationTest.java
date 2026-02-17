package org.curtinfrc.frc2026.util.Repulsor.Offload.Processor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import org.junit.jupiter.api.Test;

class OffloadableProcessorGenerationTest {
  @Test
  void shouldGenerateWrapperAndAutoDtos() throws Exception {
    CompilationResult result = compileFixture();

    Path requestPath =
        result.generatedDir().resolve("test/sample/SampleEntrypoints_compute_OffloadRequest.java");
    Path responsePath =
        result.generatedDir().resolve("test/sample/SampleEntrypoints_compute_OffloadResponse.java");
    Path wrapperPath =
        result.generatedDir().resolve("test/sample/SampleEntrypoints_Offloaded.java");
    Path taskBindingPath = findTaskBindingSource(result.generatedDir());

    assertTrue(Files.exists(requestPath), "Expected generated request DTO");
    assertTrue(Files.exists(responsePath), "Expected generated response DTO");
    assertTrue(Files.exists(wrapperPath), "Expected generated wrapper");
    assertTrue(Files.exists(taskBindingPath), "Expected generated task binding");

    String wrapper = Files.readString(wrapperPath);
    assertTrue(wrapper.contains("compute_offload("), "Wrapper should expose sync offload method");
    assertTrue(
        wrapper.contains("compute_offloadAsync("), "Wrapper should expose async offload method");
    assertTrue(
        wrapper.contains("compute_offloadExecute("),
        "Wrapper should expose server execute bridge method");
  }

  @Test
  void shouldGenerateStableManifestContent() throws Exception {
    CompilationResult first = compileFixture();
    CompilationResult second = compileFixture();

    String firstManifest =
        Files.readString(first.classOutputDir().resolve("offload-manifest.json"));
    String secondManifest =
        Files.readString(second.classOutputDir().resolve("offload-manifest.json"));

    assertEquals(firstManifest, secondManifest, "Manifest output should be deterministic");
    assertTrue(
        firstManifest.contains("\"id\": \"test.sample.compute.v2\""),
        "Manifest should contain annotated task ID");
    assertTrue(
        firstManifest.contains(
            "\"requestType\": \"test.sample.SampleEntrypoints_compute_OffloadRequest\""),
        "Manifest should reference generated request DTO");
    assertTrue(
        firstManifest.contains(
            "\"responseType\": \"test.sample.SampleEntrypoints_compute_OffloadResponse\""),
        "Manifest should reference generated response DTO");

    String serviceDescriptor =
        Files.readString(
            first
                .classOutputDir()
                .resolve(
                    "META-INF/services/org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction"));
    String bindingClassName =
        "test.sample." + first.taskBindingSource().getFileName().toString().replace(".java", "");
    assertTrue(
        serviceDescriptor.contains(bindingClassName),
        "Service descriptor should include generated task binding");
  }

  private static CompilationResult compileFixture() throws IOException {
    JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
    if (compiler == null) {
      throw new IllegalStateException("System JavaCompiler not available");
    }

    Path tempDir = Files.createTempDirectory("offload-processor-test");
    Path sourceDir = tempDir.resolve("src");
    Path generatedDir = tempDir.resolve("generated");
    Path classOutputDir = tempDir.resolve("classes");
    Files.createDirectories(sourceDir.resolve("test/sample"));
    Files.createDirectories(generatedDir);
    Files.createDirectories(classOutputDir);

    Path sourcePath = sourceDir.resolve("test/sample/SampleEntrypoints.java");
    Files.writeString(
        sourcePath,
        """
        package test.sample;

        import org.curtinfrc.frc2026.util.Repulsor.Offload.Offloadable;

        public final class SampleEntrypoints {
          private SampleEntrypoints() {}

          @Offloadable(id = "test.sample.compute.v2", version = 2, timeoutMs = 17, fallback = true)
          public static int compute(int a, double b) {
            return (int) (a + b);
          }
        }
        """);

    DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
    List<String> options = new ArrayList<>();
    options.add("-classpath");
    options.add(System.getProperty("java.class.path"));
    options.add("-processor");
    options.add(OffloadableProcessor.class.getName());
    options.add("-d");
    options.add(classOutputDir.toString());
    options.add("-s");
    options.add(generatedDir.toString());

    try (StandardJavaFileManager fileManager =
        compiler.getStandardFileManager(diagnostics, null, null)) {
      Iterable<? extends JavaFileObject> sources =
          fileManager.getJavaFileObjects(sourcePath.toFile());

      Boolean success =
          compiler.getTask(null, fileManager, diagnostics, options, null, sources).call();
      if (success == null || !success) {
        throw new AssertionError("Compilation failed:\n" + formatDiagnostics(diagnostics));
      }
    }

    return new CompilationResult(generatedDir, classOutputDir, findTaskBindingSource(generatedDir));
  }

  private static String formatDiagnostics(DiagnosticCollector<JavaFileObject> diagnostics) {
    StringBuilder output = new StringBuilder();
    for (Diagnostic<? extends JavaFileObject> diagnostic : diagnostics.getDiagnostics()) {
      output.append(diagnostic.getKind()).append(": ").append(diagnostic.getMessage(null));
      output.append(System.lineSeparator());
    }
    return output.toString();
  }

  private static Path findTaskBindingSource(Path generatedDir) throws IOException {
    Path folder = generatedDir.resolve("test/sample");
    try (Stream<Path> paths = Files.list(folder)) {
      return paths
          .filter(path -> path.getFileName().toString().endsWith("_OffloadTaskBinding.java"))
          .findFirst()
          .orElseThrow(() -> new AssertionError("Expected generated task binding source"));
    }
  }

  private record CompilationResult(
      Path generatedDir, Path classOutputDir, Path taskBindingSource) {}
}
