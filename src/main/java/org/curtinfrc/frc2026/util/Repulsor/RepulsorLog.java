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

