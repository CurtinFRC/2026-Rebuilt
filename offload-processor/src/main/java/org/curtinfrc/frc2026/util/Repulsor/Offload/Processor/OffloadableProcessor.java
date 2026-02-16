package org.curtinfrc.frc2026.util.Repulsor.Offload.Processor;

import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Filer;
import javax.annotation.processing.Messager;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.Elements;
import javax.lang.model.util.Types;
import javax.tools.Diagnostic;
import javax.tools.FileObject;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Offloadable;

@SupportedAnnotationTypes("org.curtinfrc.frc2026.util.Repulsor.Offload.Offloadable")
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public final class OffloadableProcessor extends AbstractProcessor {
  private final Map<String, List<MethodModel>> modelsByDeclaringClass = new LinkedHashMap<>();
  private final Set<String> generatedWrappers = new HashSet<>();
  private boolean generatedManifest;

  private Filer filer;
  private Messager messager;
  private Elements elements;
  private Types types;

  @Override
  public synchronized void init(ProcessingEnvironment processingEnv) {
    super.init(processingEnv);
    filer = processingEnv.getFiler();
    messager = processingEnv.getMessager();
    elements = processingEnv.getElementUtils();
    types = processingEnv.getTypeUtils();
  }

  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    for (Element element : roundEnv.getElementsAnnotatedWith(Offloadable.class)) {
      if (element.getKind() != ElementKind.METHOD) {
        error(element, "@Offloadable can only be applied to methods");
        continue;
      }

      ExecutableElement method = (ExecutableElement) element;
      MethodModel model = parseMethod(method);
      if (model == null) {
        continue;
      }
      modelsByDeclaringClass
          .computeIfAbsent(model.declaringClassName(), unused -> new ArrayList<>())
          .add(model);
    }

    try {
      generatePendingWrappers();
    } catch (IOException ex) {
      error(null, "Failed to generate offload wrappers: " + ex.getMessage());
    }

    if (roundEnv.processingOver() && !generatedManifest) {
      generatedManifest = true;
      try {
        generateManifest();
        generateIdsClass();
      } catch (IOException ex) {
        error(null, "Failed to generate offload manifest and IDs: " + ex.getMessage());
      }
    }

    return false;
  }

  private void generatePendingWrappers() throws IOException {
    for (Map.Entry<String, List<MethodModel>> entry : modelsByDeclaringClass.entrySet()) {
      String declaringClassName = entry.getKey();
      if (generatedWrappers.contains(declaringClassName)) {
        continue;
      }
      generateWrapperForClass(entry.getValue());
      generatedWrappers.add(declaringClassName);
    }
  }

  private MethodModel parseMethod(ExecutableElement method) {
    TypeElement enclosingType = (TypeElement) method.getEnclosingElement();
    PackageElement pkg = elements.getPackageOf(enclosingType);

    if (!method.getModifiers().contains(Modifier.STATIC)) {
      error(method, "@Offloadable method must be static");
      return null;
    }

    if (method.getParameters().size() != 1) {
      error(method, "@Offloadable methods currently must accept exactly one request DTO parameter");
      return null;
    }

    if (method.getReturnType().getKind() == TypeKind.VOID) {
      error(method, "@Offloadable method must return a non-void response DTO");
      return null;
    }

    if (method.getReturnType().getKind().isPrimitive()) {
      error(method, "@Offloadable method return type must be a DTO class, not a primitive");
      return null;
    }

    AnnotationValues annotationValues = readAnnotationValues(method);
    VariableElement requestParameter = method.getParameters().get(0);

    String declaringClassName = enclosingType.getQualifiedName().toString();
    String requestType = requestParameter.asType().toString();
    String responseType = method.getReturnType().toString();
    String signature = requestType + "->" + responseType;
    String id =
        annotationValues.id().isBlank()
            ? OffloadIdGenerator.autoId(
                declaringClassName,
                method.getSimpleName().toString(),
                signature,
                annotationValues.version())
            : annotationValues.id();

    TypeMirror erasedResponseType = types.erasure(method.getReturnType());

    return new MethodModel(
        pkg.getQualifiedName().toString(),
        enclosingType.getSimpleName().toString(),
        declaringClassName,
        method.getSimpleName().toString(),
        requestParameter.getSimpleName().toString(),
        requestType,
        responseType,
        erasedResponseType.toString(),
        id,
        annotationValues.version(),
        annotationValues.timeoutMs(),
        annotationValues.fallback());
  }

  private AnnotationValues readAnnotationValues(ExecutableElement method) {
    Map<String, Object> values = new HashMap<>();
    for (AnnotationMirror mirror : method.getAnnotationMirrors()) {
      if (!mirror.getAnnotationType().toString().equals(Offloadable.class.getName())) {
        continue;
      }
      for (Map.Entry<? extends ExecutableElement, ? extends AnnotationValue> entry :
          mirror.getElementValues().entrySet()) {
        values.put(entry.getKey().getSimpleName().toString(), entry.getValue().getValue());
      }
    }

    String id = (String) values.getOrDefault("id", "");
    int version = ((Number) values.getOrDefault("version", 1)).intValue();
    int timeoutMs = ((Number) values.getOrDefault("timeoutMs", 20)).intValue();
    boolean fallback = (Boolean) values.getOrDefault("fallback", Boolean.TRUE);
    return new AnnotationValues(id, version, timeoutMs, fallback);
  }

  private void generateWrapperForClass(List<MethodModel> models) throws IOException {
    if (models == null || models.isEmpty()) {
      return;
    }

    MethodModel first = models.get(0);
    String packageName = first.packageName();
    String wrapperName = first.enclosingSimpleName() + "_Offloaded";
    String fqcn = packageName.isBlank() ? wrapperName : packageName + "." + wrapperName;

    JavaFileObject fileObject = filer.createSourceFile(fqcn);
    try (Writer writer = fileObject.openWriter()) {
      if (!packageName.isBlank()) {
        writer.write("package " + packageName + ";\n\n");
      }

      writer.write("public final class " + wrapperName + " {\n");
      writer.write("  private " + wrapperName + "() {}\n\n");

      for (MethodModel model : models) {
        String constantName = constantName(model);
        writer.write("  public static final String " + constantName + " = \"");
        writer.write(escape(model.id()));
        writer.write("\";\n\n");

        writer.write("  public static " + model.responseType() + " " + model.methodName() + "(");
        writer.write(model.requestType() + " " + model.requestParameterName());
        writer.write(") {\n");
        writer.write("    try {\n");
        writer.write(
            "      return " + model.methodName() + "Async(" + model.requestParameterName() + ")\n");
        writer.write(
            "          .get("
                + (model.timeoutMs() + 5)
                + "L, java.util.concurrent.TimeUnit.MILLISECONDS);\n");
        writer.write("    } catch (Exception ex) {\n");
        if (model.fallback()) {
          writer.write(
              "      return "
                  + model.declaringClassName()
                  + "."
                  + model.methodName()
                  + "("
                  + model.requestParameterName()
                  + ");\n");
        } else {
          writer.write(
              "      throw new RuntimeException(\"Offload call failed for task "
                  + escape(model.id())
                  + "\", ex);\n");
        }
        writer.write("    }\n");
        writer.write("  }\n\n");

        writer.write(
            "  public static java.util.concurrent.CompletableFuture<"
                + model.responseType()
                + "> ");
        writer.write(
            model.methodName()
                + "Async("
                + model.requestType()
                + " "
                + model.requestParameterName()
                + ") {\n");
        writer.write(
            "    java.util.concurrent.CompletableFuture<" + model.responseType() + "> remote =\n");
        writer.write(
            "        org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc.callTyped("
                + constantName
                + ", "
                + model.requestParameterName()
                + ", "
                + model.erasedResponseType()
                + ".class, "
                + model.timeoutMs()
                + ");\n");
        if (!model.fallback()) {
          writer.write("    return remote;\n");
          writer.write("  }\n\n");
          continue;
        }
        writer.write("    return remote.handle((value, error) -> {\n");
        writer.write("      if (error == null) {\n");
        writer.write(
            "        return java.util.concurrent.CompletableFuture.completedFuture(value);\n");
        writer.write("      }\n");
        writer.write(
            "      return java.util.concurrent.CompletableFuture.supplyAsync(() -> "
                + model.declaringClassName()
                + "."
                + model.methodName()
                + "("
                + model.requestParameterName()
                + "), "
                + "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc.fallbackExecutor());\n");
        writer.write("    }).thenCompose(future -> future);\n");
        writer.write("  }\n\n");
      }

      writer.write("}\n");
    }
  }

  private void generateManifest() throws IOException {
    List<MethodModel> methods = allMethodsSorted();
    FileObject fileObject =
        filer.createResource(StandardLocation.CLASS_OUTPUT, "", "offload-manifest.json");
    try (Writer writer = fileObject.openWriter()) {
      writer.write("{\n");
      writer.write("  \"schemaVersion\": 1,\n");
      writer.write("  \"functions\": [\n");
      for (int i = 0; i < methods.size(); i++) {
        MethodModel model = methods.get(i);
        writer.write("    {\n");
        writer.write("      \"id\": \"");
        writer.write(escape(model.id()));
        writer.write("\",\n");
        writer.write("      \"declaringClass\": \"");
        writer.write(escape(model.declaringClassName()));
        writer.write("\",\n");
        writer.write("      \"methodName\": \"");
        writer.write(escape(model.methodName()));
        writer.write("\",\n");
        writer.write("      \"requestType\": \"");
        writer.write(escape(model.requestType()));
        writer.write("\",\n");
        writer.write("      \"responseType\": \"");
        writer.write(escape(model.responseType()));
        writer.write("\",\n");
        writer.write("      \"version\": " + model.version() + ",\n");
        writer.write("      \"timeoutMs\": " + model.timeoutMs() + ",\n");
        writer.write("      \"fallback\": " + model.fallback() + "\n");
        writer.write("    }");
        writer.write(i == methods.size() - 1 ? "\n" : ",\n");
      }
      writer.write("  ]\n");
      writer.write("}\n");
    }
  }

  private void generateIdsClass() throws IOException {
    List<MethodModel> methods = allMethodsSorted();
    if (methods.isEmpty()) {
      return;
    }

    String packageName = "org.curtinfrc.frc2026.util.Repulsor.Offload.Generated";
    String className = "OffloadIds";
    JavaFileObject fileObject = filer.createSourceFile(packageName + "." + className);
    try (Writer writer = fileObject.openWriter()) {
      writer.write("package " + packageName + ";\n\n");
      writer.write("public final class " + className + " {\n");
      writer.write("  private " + className + "() {}\n\n");
      for (MethodModel model : methods) {
        writer.write("  public static final String " + constantName(model) + " = \"");
        writer.write(escape(model.id()));
        writer.write("\";\n");
      }
      writer.write("}\n");
    }
  }

  private List<MethodModel> allMethodsSorted() {
    List<MethodModel> methods = new ArrayList<>();
    for (List<MethodModel> value : modelsByDeclaringClass.values()) {
      methods.addAll(value);
    }
    methods.sort(Comparator.comparing(MethodModel::id));
    return methods;
  }

  private String constantName(MethodModel model) {
    String base = model.enclosingSimpleName() + "_" + model.methodName() + "_" + model.version();
    return base.replaceAll("[^A-Za-z0-9]", "_").toUpperCase();
  }

  private static String escape(String input) {
    return input.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private void error(Element element, String message) {
    messager.printMessage(Diagnostic.Kind.ERROR, message, element);
  }

  private record AnnotationValues(String id, int version, int timeoutMs, boolean fallback) {}

  private record MethodModel(
      String packageName,
      String enclosingSimpleName,
      String declaringClassName,
      String methodName,
      String requestParameterName,
      String requestType,
      String responseType,
      String erasedResponseType,
      String id,
      int version,
      int timeoutMs,
      boolean fallback) {}
}
