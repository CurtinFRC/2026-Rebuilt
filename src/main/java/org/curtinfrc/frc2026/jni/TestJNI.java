package org.curtinfrc.frc2026.jni;

import edu.wpi.first.util.RuntimeLoader;
import java.io.IOException;

public class TestJNI {
  public static void load() throws IOException {
    RuntimeLoader.loadLibrary("testJNI");
  }

  public static native void helloWorld();
}
