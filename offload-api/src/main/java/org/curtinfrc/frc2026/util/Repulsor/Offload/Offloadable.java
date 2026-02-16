package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.METHOD)
@Retention(RetentionPolicy.SOURCE)
public @interface Offloadable {
  String id() default "";

  int version() default 1;

  int timeoutMs() default 20;

  boolean fallback() default true;
}
