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
  private final Set<String> discoveredMethodKeys = new HashSet<>();
  private final Set<String> generatedWrappers = new HashSet<>();
  private final Set<String> generatedDtos = new HashSet<>();
  private final Set<String> generatedTaskBindings = new HashSet<>();
  private boolean generatedManifest;
  private boolean generatedTaskService;

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

      if (!discoveredMethodKeys.add(model.methodKey())) {
        continue;
      }

      modelsByDeclaringClass
          .computeIfAbsent(model.declaringClassName(), unused -> new ArrayList<>())
          .add(model);
    }

    try {
      generatePendingArtifacts();
    } catch (IOException ex) {
      error(null, "Failed to generate offload artifacts: " + ex.getMessage());
    }

    if (roundEnv.processingOver() && !generatedManifest) {
      generatedManifest = true;
      try {
        generateManifest();
        generateIdsClass();
        generateTaskServiceFile();
      } catch (IOException ex) {
        error(null, "Failed to generate offload manifest artifacts: " + ex.getMessage());
      }
    }

    return false;
  }

  private void generatePendingArtifacts() throws IOException {
    for (Map.Entry<String, List<MethodModel>> entry : modelsByDeclaringClass.entrySet()) {
      String declaringClass = entry.getKey();
      List<MethodModel> methods = entry.getValue();

      for (MethodModel method : methods) {
        generateDtoClassIfNeeded(
            method.requestFqcn(), method.requestSimpleName(), method.parameters());
        generateResponseDtoClassIfNeeded(method.responseFqcn(), method.responseSimpleName());
        generateTaskBindingClassIfNeeded(method);
      }

      if (!generatedWrappers.contains(declaringClass)) {
        generateWrapperForClass(methods);
        generatedWrappers.add(declaringClass);
      }
    }
  }

  private MethodModel parseMethod(ExecutableElement method) {
    TypeElement enclosingType = (TypeElement) method.getEnclosingElement();
    PackageElement pkg = elements.getPackageOf(enclosingType);

    if (!method.getModifiers().contains(Modifier.STATIC)) {
      error(method, "@Offloadable method must be static");
      return null;
    }

    if (method.getReturnType().getKind() == TypeKind.VOID) {
      error(method, "@Offloadable method must return a value");
      return null;
    }

    AnnotationValues annotationValues = readAnnotationValues(method);

    List<ParamModel> params = new ArrayList<>();
    for (int i = 0; i < method.getParameters().size(); i++) {
      VariableElement parameter = method.getParameters().get(i);
      params.add(
          new ParamModel(
              parameter.getSimpleName().toString(),
              parameter.asType().toString(),
              parameter.asType().getKind().isPrimitive()));
    }

    String declaringClassName = enclosingType.getQualifiedName().toString();
    String packageName = pkg.getQualifiedName().toString();
    String methodName = method.getSimpleName().toString();

    String signature = buildSignature(params, method.getReturnType().toString());
    String id =
        annotationValues.id().isBlank()
            ? OffloadIdGenerator.autoId(
                declaringClassName, methodName, signature, annotationValues.version())
            : annotationValues.id();

    String suffix = Integer.toHexString(Math.abs(signature.hashCode()));
    String requestSimpleName = enclosingType.getSimpleName() + "_" + methodName + "_OffloadRequest";
    String responseSimpleName =
        enclosingType.getSimpleName() + "_" + methodName + "_OffloadResponse";
    String taskBindingSimpleName =
        enclosingType.getSimpleName() + "_" + methodName + "_" + suffix + "_OffloadTaskBinding";

    String requestFqcn =
        packageName.isBlank() ? requestSimpleName : packageName + "." + requestSimpleName;
    String responseFqcn =
        packageName.isBlank() ? responseSimpleName : packageName + "." + responseSimpleName;
    String taskBindingFqcn =
        packageName.isBlank() ? taskBindingSimpleName : packageName + "." + taskBindingSimpleName;

    return new MethodModel(
        packageName,
        enclosingType.getSimpleName().toString(),
        declaringClassName,
        methodName,
        method.getReturnType().toString(),
        method.getReturnType().getKind().isPrimitive(),
        boxedType(
            method.getReturnType().toString(), method.getReturnType().getKind().isPrimitive()),
        params,
        id,
        annotationValues.version(),
        annotationValues.timeoutMs(),
        annotationValues.fallback(),
        suffix,
        requestSimpleName,
        requestFqcn,
        responseSimpleName,
        responseFqcn,
        taskBindingSimpleName,
        taskBindingFqcn,
        declaringClassName + "#" + methodName + "(" + signature + ")");
  }

  private static String buildSignature(List<ParamModel> params, String returnType) {
    StringBuilder output = new StringBuilder();
    for (int i = 0; i < params.size(); i++) {
      if (i > 0) {
        output.append(',');
      }
      output.append(params.get(i).typeName());
    }
    output.append("->").append(returnType);
    return output.toString();
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

  private void generateDtoClassIfNeeded(String fqcn, String simpleName, List<ParamModel> params)
      throws IOException {
    if (!generatedDtos.add(fqcn)) {
      return;
    }

    String packageName = packageNameFor(fqcn, simpleName);
    JavaFileObject fileObject = filer.createSourceFile(fqcn);
    try (Writer writer = fileObject.openWriter()) {
      if (!packageName.isBlank()) {
        writer.write("package " + packageName + ";\n\n");
      }

      writer.write("public class " + simpleName + " {\n");
      for (int i = 0; i < params.size(); i++) {
        writer.write("  private byte[] arg" + i + ";\n");
      }
      writer.write("\n");
      writer.write("  public " + simpleName + "() {}\n\n");

      for (int i = 0; i < params.size(); i++) {
        writer.write("  public byte[] getArg" + i + "() {\n");
        writer.write("    return arg" + i + ";\n");
        writer.write("  }\n\n");

        writer.write("  public void setArg" + i + "(byte[] arg" + i + ") {\n");
        writer.write("    this.arg" + i + " = arg" + i + ";\n");
        writer.write("  }\n\n");
      }
      writer.write("}\n");
    }
  }

  private void generateResponseDtoClassIfNeeded(String fqcn, String simpleName) throws IOException {
    if (!generatedDtos.add(fqcn)) {
      return;
    }

    String packageName = packageNameFor(fqcn, simpleName);
    JavaFileObject fileObject = filer.createSourceFile(fqcn);
    try (Writer writer = fileObject.openWriter()) {
      if (!packageName.isBlank()) {
        writer.write("package " + packageName + ";\n\n");
      }

      writer.write("public class " + simpleName + " {\n");
      writer.write("  private byte[] result;\n\n");
      writer.write("  public " + simpleName + "() {}\n\n");
      writer.write("  public byte[] getResult() {\n");
      writer.write("    return result;\n");
      writer.write("  }\n\n");
      writer.write("  public void setResult(byte[] result) {\n");
      writer.write("    this.result = result;\n");
      writer.write("  }\n");
      writer.write("}\n");
    }
  }

  private static String packageNameFor(String fqcn, String simpleName) {
    if (fqcn.equals(simpleName)) {
      return "";
    }
    return fqcn.substring(0, fqcn.length() - (simpleName.length() + 1));
  }

  private void generateTaskBindingClassIfNeeded(MethodModel method) throws IOException {
    if (!generatedTaskBindings.add(method.taskBindingFqcn())) {
      return;
    }

    String packageName = method.packageName();
    JavaFileObject fileObject = filer.createSourceFile(method.taskBindingFqcn());
    try (Writer writer = fileObject.openWriter()) {
      if (!packageName.isBlank()) {
        writer.write("package " + packageName + ";\n\n");
      }

      writer.write(
          "public final class "
              + method.taskBindingSimpleName()
              + " implements org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction<"
              + method.requestSimpleName()
              + ", "
              + method.responseSimpleName()
              + "> {\n");

      writer.write("  @Override\n");
      writer.write("  public String taskId() {\n");
      writer.write("    return \"" + escape(method.id()) + "\";\n");
      writer.write("  }\n\n");

      writer.write("  @Override\n");
      writer.write("  public Class<" + method.requestSimpleName() + "> requestType() {\n");
      writer.write("    return " + method.requestSimpleName() + ".class;\n");
      writer.write("  }\n\n");

      writer.write("  @Override\n");
      writer.write("  public Class<" + method.responseSimpleName() + "> responseType() {\n");
      writer.write("    return " + method.responseSimpleName() + ".class;\n");
      writer.write("  }\n\n");

      writer.write("  @Override\n");
      writer.write("  public int timeoutMs() {\n");
      writer.write("    return " + method.timeoutMs() + ";\n");
      writer.write("  }\n\n");

      writer.write("  @Override\n");
      writer.write(
          "  public "
              + method.responseSimpleName()
              + " execute("
              + method.requestSimpleName()
              + " request) {\n");
      writer.write(
          "    return "
              + method.enclosingSimpleName()
              + "_Offloaded."
              + method.methodName()
              + "_offloadExecute(request);\n");
      writer.write("  }\n");
      writer.write("}\n");
    }
  }

  private void generateWrapperForClass(List<MethodModel> methods) throws IOException {
    if (methods == null || methods.isEmpty()) {
      return;
    }

    MethodModel first = methods.get(0);
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

      methods.sort(Comparator.comparing(MethodModel::methodName));
      for (MethodModel method : methods) {
        writeMethodArtifacts(writer, method);
      }

      writer.write("}\n");
    }
  }

  private void writeMethodArtifacts(Writer writer, MethodModel method) throws IOException {
    String constantName = constantName(method);
    writer.write("  public static final String " + constantName + " = \"");
    writer.write(escape(method.id()));
    writer.write("\";\n\n");

    String argsDecl = argsDeclaration(method.parameters());
    String argsInvoke = argsInvocation(method.parameters());

    writer.write(
        "  public static "
            + method.returnType()
            + " "
            + method.methodName()
            + "_offload("
            + argsDecl
            + ") {\n");
    writer.write("    try {\n");
    writer.write(
        "      return "
            + method.methodName()
            + "_offloadAsync("
            + argsInvoke
            + ").get("
            + (method.timeoutMs() + 5)
            + "L, java.util.concurrent.TimeUnit.MILLISECONDS);\n");
    writer.write("    } catch (Exception ex) {\n");
    if (method.fallback()) {
      writer.write(
          "      return "
              + method.declaringClassName()
              + "."
              + method.methodName()
              + "("
              + argsInvoke
              + ");\n");
    } else {
      writer.write(
          "      throw new RuntimeException(\"Offload call failed for task "
              + escape(method.id())
              + "\", ex);\n");
    }
    writer.write("    }\n");
    writer.write("  }\n\n");

    writer.write(
        "  public static java.util.concurrent.CompletableFuture<"
            + method.boxedReturnType()
            + "> "
            + method.methodName()
            + "_offloadAsync("
            + argsDecl
            + ") {\n");
    writer.write(
        "    org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadBootstrap.ensureInitialized();\n");
    writer.write(
        "    if (!org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc.gateway().isHealthy()) {\n");
    if (method.fallback()) {
      writer.write(
          "      return java.util.concurrent.CompletableFuture.completedFuture("
              + method.declaringClassName()
              + "."
              + method.methodName()
              + "("
              + argsInvoke
              + "));\n");
    } else {
      writer.write(
          "      return java.util.concurrent.CompletableFuture.failedFuture(new IllegalStateException(\"Offload gateway unavailable\"));\n");
    }
    writer.write("    }\n\n");

    writer.write(
        "    "
            + method.requestSimpleName()
            + " request = new "
            + method.requestSimpleName()
            + "();\n");
    for (int i = 0; i < method.parameters().size(); i++) {
      ParamModel param = method.parameters().get(i);
      writer.write(
          "    request.setArg"
              + i
              + "(org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadValueCodec.encode(\""
              + escape(param.typeName())
              + "\", "
              + param.name()
              + "));\n");
    }

    writer.write(
        "    java.util.concurrent.CompletableFuture<"
            + method.responseSimpleName()
            + "> remote =\n");
    writer.write(
        "        org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc.callTyped("
            + constantName
            + ", request, "
            + method.responseSimpleName()
            + ".class, "
            + method.timeoutMs()
            + ");\n");

    writer.write(
        "    java.util.concurrent.CompletableFuture<"
            + method.boxedReturnType()
            + "> decoded = remote.thenApply(response -> "
            + decodeReturnExpr(method, "response.getResult()")
            + ");\n");

    if (!method.fallback()) {
      writer.write("    return decoded;\n");
      writer.write("  }\n\n");
    } else {
      writer.write("    return decoded.handle((value, error) -> {\n");
      writer.write("      if (error == null) {\n");
      writer.write(
          "        return java.util.concurrent.CompletableFuture.completedFuture(value);\n");
      writer.write("      }\n");
      writer.write(
          "      return java.util.concurrent.CompletableFuture.supplyAsync(() -> "
              + method.declaringClassName()
              + "."
              + method.methodName()
              + "("
              + argsInvoke
              + "), org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc.fallbackExecutor());\n");
      writer.write("    }).thenCompose(future -> future);\n");
      writer.write("  }\n\n");
    }

    writer.write(
        "  public static "
            + method.responseSimpleName()
            + " "
            + method.methodName()
            + "_offloadExecute("
            + method.requestSimpleName()
            + " request) {\n");
    for (int i = 0; i < method.parameters().size(); i++) {
      ParamModel param = method.parameters().get(i);
      writer.write(
          "    "
              + param.typeName()
              + " "
              + param.name()
              + " = "
              + decodeParamExpr(param, "request.getArg" + i + "()")
              + ";\n");
    }
    writer.write(
        "    "
            + method.returnType()
            + " result = "
            + method.declaringClassName()
            + "."
            + method.methodName()
            + "("
            + argsInvoke
            + ");\n");
    writer.write(
        "    "
            + method.responseSimpleName()
            + " response = new "
            + method.responseSimpleName()
            + "();\n");
    writer.write(
        "    response.setResult(org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadValueCodec.encode(\""
            + escape(method.returnType())
            + "\", result));\n");
    writer.write("    return response;\n");
    writer.write("  }\n\n");
  }

  private static String argsDeclaration(List<ParamModel> params) {
    StringBuilder output = new StringBuilder();
    for (int i = 0; i < params.size(); i++) {
      if (i > 0) {
        output.append(", ");
      }
      output.append(params.get(i).typeName()).append(' ').append(params.get(i).name());
    }
    return output.toString();
  }

  private static String argsInvocation(List<ParamModel> params) {
    StringBuilder output = new StringBuilder();
    for (int i = 0; i < params.size(); i++) {
      if (i > 0) {
        output.append(", ");
      }
      output.append(params.get(i).name());
    }
    return output.toString();
  }

  private static String decodeParamExpr(ParamModel param, String payloadExpr) {
    String decodeCall =
        "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadValueCodec.decode(\""
            + escape(param.typeName())
            + "\", "
            + payloadExpr
            + ")";
    if (!param.primitive()) {
      return "(" + param.typeName() + ") " + decodeCall;
    }

    return switch (param.typeName()) {
      case "byte" -> "((java.lang.Byte) " + decodeCall + ").byteValue()";
      case "short" -> "((java.lang.Short) " + decodeCall + ").shortValue()";
      case "int" -> "((java.lang.Integer) " + decodeCall + ").intValue()";
      case "long" -> "((java.lang.Long) " + decodeCall + ").longValue()";
      case "float" -> "((java.lang.Float) " + decodeCall + ").floatValue()";
      case "double" -> "((java.lang.Double) " + decodeCall + ").doubleValue()";
      case "boolean" -> "((java.lang.Boolean) " + decodeCall + ").booleanValue()";
      case "char" -> "((java.lang.Character) " + decodeCall + ").charValue()";
      default -> throw new IllegalStateException("Unsupported primitive type: " + param.typeName());
    };
  }

  private static String decodeReturnExpr(MethodModel method, String payloadExpr) {
    String decodeCall =
        "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadValueCodec.decode(\""
            + escape(method.returnType())
            + "\", "
            + payloadExpr
            + ")";

    if (!method.returnPrimitive()) {
      return "(" + method.boxedReturnType() + ") " + decodeCall;
    }

    return switch (method.returnType()) {
      case "byte" -> "((java.lang.Byte) " + decodeCall + ")";
      case "short" -> "((java.lang.Short) " + decodeCall + ")";
      case "int" -> "((java.lang.Integer) " + decodeCall + ")";
      case "long" -> "((java.lang.Long) " + decodeCall + ")";
      case "float" -> "((java.lang.Float) " + decodeCall + ")";
      case "double" -> "((java.lang.Double) " + decodeCall + ")";
      case "boolean" -> "((java.lang.Boolean) " + decodeCall + ")";
      case "char" -> "((java.lang.Character) " + decodeCall + ")";
      default ->
          throw new IllegalStateException(
              "Unsupported primitive return type: " + method.returnType());
    };
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
        MethodModel method = methods.get(i);
        writer.write("    {\n");
        writer.write("      \"id\": \"");
        writer.write(escape(method.id()));
        writer.write("\",\n");
        writer.write("      \"declaringClass\": \"");
        writer.write(escape(method.declaringClassName()));
        writer.write("\",\n");
        writer.write("      \"methodName\": \"");
        writer.write(escape(method.methodName()));
        writer.write("\",\n");
        writer.write("      \"requestType\": \"");
        writer.write(escape(method.requestFqcn()));
        writer.write("\",\n");
        writer.write("      \"responseType\": \"");
        writer.write(escape(method.responseFqcn()));
        writer.write("\",\n");
        writer.write("      \"version\": " + method.version() + ",\n");
        writer.write("      \"timeoutMs\": " + method.timeoutMs() + ",\n");
        writer.write("      \"fallback\": " + method.fallback() + "\n");
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
      for (MethodModel method : methods) {
        writer.write("  public static final String " + constantName(method) + " = \"");
        writer.write(escape(method.id()));
        writer.write("\";\n");
      }
      writer.write("}\n");
    }
  }

  private void generateTaskServiceFile() throws IOException {
    if (generatedTaskService) {
      return;
    }
    generatedTaskService = true;

    List<MethodModel> methods = allMethodsSorted();
    if (methods.isEmpty()) {
      return;
    }

    FileObject serviceFile =
        filer.createResource(
            StandardLocation.CLASS_OUTPUT,
            "",
            "META-INF/services/org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction");

    try (Writer writer = serviceFile.openWriter()) {
      for (MethodModel method : methods) {
        writer.write(method.taskBindingFqcn());
        writer.write("\n");
      }
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

  private static String constantName(MethodModel method) {
    String base =
        method.enclosingSimpleName()
            + "_"
            + method.methodName()
            + "_"
            + method.suffix()
            + "_"
            + method.version();
    return base.replaceAll("[^A-Za-z0-9]", "_").toUpperCase();
  }

  private static String boxedType(String typeName, boolean primitive) {
    if (!primitive) {
      return typeName;
    }
    return switch (typeName) {
      case "byte" -> "java.lang.Byte";
      case "short" -> "java.lang.Short";
      case "int" -> "java.lang.Integer";
      case "long" -> "java.lang.Long";
      case "float" -> "java.lang.Float";
      case "double" -> "java.lang.Double";
      case "boolean" -> "java.lang.Boolean";
      case "char" -> "java.lang.Character";
      default -> throw new IllegalStateException("Unsupported primitive type: " + typeName);
    };
  }

  private static String escape(String input) {
    return input.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private void error(Element element, String message) {
    messager.printMessage(Diagnostic.Kind.ERROR, message, element);
  }

  private record AnnotationValues(String id, int version, int timeoutMs, boolean fallback) {}

  private record ParamModel(String name, String typeName, boolean primitive) {}

  private record MethodModel(
      String packageName,
      String enclosingSimpleName,
      String declaringClassName,
      String methodName,
      String returnType,
      boolean returnPrimitive,
      String boxedReturnType,
      List<ParamModel> parameters,
      String id,
      int version,
      int timeoutMs,
      boolean fallback,
      String suffix,
      String requestSimpleName,
      String requestFqcn,
      String responseSimpleName,
      String responseFqcn,
      String taskBindingSimpleName,
      String taskBindingFqcn,
      String methodKey) {}
}
