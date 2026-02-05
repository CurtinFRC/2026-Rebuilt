/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RepulsorLog {
  public static enum LogType {
    kSYSOUT,
    kNT4
  }

  private static LogType logType = LogType.kNT4;
  private static boolean enabled = true;

  public static void setLogType(LogType type) {
    logType = type;
  }

  public static void setEnabled(boolean enable) {
    enabled = enable;
  }

  public static void log(String key, String value) {
    if (!enabled) {
      return;
    }
    switch (logType) {
      case kSYSOUT:
        System.out.println(key + ": " + value);
        break;
      case kNT4:
        NetworkTableInstance.getDefault().getTable("Repulsor").getEntry(key).setString(value);
        break;
      default:
        System.err.println("Unknown log type: " + logType);
    }
  }

  public static void log(String key, Pose2d value) {
    if (!enabled) {
      return;
    }

    switch (logType) {
      case kSYSOUT:
        System.out.println(key + ": " + value.toString());
        break;
      case kNT4:
        NetworkTableInstance.getDefault()
            .getTable("Repulsor")
            .getEntry(key + "/translation/x")
            .setDouble(value.getX());
        NetworkTableInstance.getDefault()
            .getTable("Repulsor")
            .getEntry(key + "/translation/y")
            .setDouble(value.getY());
        NetworkTableInstance.getDefault()
            .getTable("Repulsor")
            .getEntry(key + "/rotation/value")
            .setDouble(value.getRotation().getRadians());
        break;
      default:
        System.err.println("Unknown log type: " + logType);
    }
  }

  public static void log(String message) {
    if (!enabled) {
      return;
    }
    switch (logType) {
      case kSYSOUT:
        System.out.println(message);
        break;
      case kNT4:
        String previousLogs =
            NetworkTableInstance.getDefault()
                .getTable("RepulsorLogs")
                .getEntry("Logs")
                .getString("");
        String newLogs = previousLogs + message + "\n";
        NetworkTableInstance.getDefault().getTable("Repulsor").getEntry("Logs").setString(newLogs);
        break;
      default:
        System.err.println("Unknown log type: " + logType);
    }
  }

  public static void log(String message, LogType type) {
    if (!enabled) {
      return;
    }
    switch (type) {
      case kSYSOUT:
        System.out.println(message);
        break;
      case kNT4:
        String previousLogs =
            NetworkTableInstance.getDefault()
                .getTable("RepulsorLogs")
                .getEntry("Logs")
                .getString("");
        String newLogs = previousLogs + message + "\n";
        NetworkTableInstance.getDefault().getTable("Repulsor").getEntry("Logs").setString(newLogs);
        break;
      default:
        System.err.println("Unknown log type: " + logType);
    }
  }
}
