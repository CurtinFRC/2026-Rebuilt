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

package org.curtinfrc.frc2026.util.Repulsor.Profiler;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.LongAdder;
import java.util.zip.GZIPOutputStream;

final class ProfilerEventWriter implements AutoCloseable {
  private final ArrayBlockingQueue<String> q;
  private final Thread thread;
  private final BufferedWriter out;
  private volatile boolean running = true;
  private final LongAdder dropped = new LongAdder();
  private final Path path;

  ProfilerEventWriter(int queueCapacity, boolean gzip) {
    this.q = new ArrayBlockingQueue<>(queueCapacity);
    this.path = makePath(gzip);
    try {
      Files.createDirectories(this.path.getParent());
      OutputStream os =
          Files.newOutputStream(
              this.path,
              StandardOpenOption.CREATE,
              StandardOpenOption.TRUNCATE_EXISTING,
              StandardOpenOption.WRITE);
      if (gzip) {
        os = new GZIPOutputStream(os, 64 * 1024, true);
      }
      this.out = new BufferedWriter(new OutputStreamWriter(os, StandardCharsets.UTF_8), 64 * 1024);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    this.thread =
        new Thread(
            () -> {
              try {
                loop();
              } catch (Throwable ignored) {
              } finally {
                try {
                  out.flush();
                } catch (Throwable ignored2) {
                }
                try {
                  out.close();
                } catch (Throwable ignored3) {
                }
              }
            },
            "RepulsorProfilerWriter");
    this.thread.setDaemon(true);
    this.thread.start();
    writeRaw("{\"t\":\"m\",\"ts\":" + nowMs() + ",\"f\":\"" + escape(path.toString()) + "\"}");
  }

  Path path() {
    return path;
  }

  void writeRaw(String line) {
    if (!running) return;
    if (!q.offer(line)) {
      dropped.increment();
    }
  }

  void flushNow() {
    writeRaw("{\"t\":\"f\",\"ts\":" + nowMs() + "}");
  }

  long droppedCount() {
    return dropped.sum();
  }

  private void loop() throws Exception {
    while (running || !q.isEmpty()) {
      String s = q.poll(200, TimeUnit.MILLISECONDS);
      if (s == null) continue;
      out.write(s);
      out.write('\n');
      int n = 0;
      while (n < 16384) {
        String s2 = q.poll();
        if (s2 == null) break;
        out.write(s2);
        out.write('\n');
        n++;
      }
      out.flush();
    }
  }

  @Override
  public void close() {
    running = false;
    try {
      thread.join(1200);
    } catch (InterruptedException ignored) {
      Thread.currentThread().interrupt();
    }
    try {
      out.flush();
    } catch (Throwable ignored) {
    }
    try {
      out.close();
    } catch (Throwable ignored) {
    }
  }

  private static long nowMs() {
    return System.currentTimeMillis();
  }

  private static Path makePath(boolean gzip) {
    String ts = Instant.now().toString().replace(':', '-');
    Path base = Filesystem.getOperatingDirectory().toPath().resolve("repulsor-profiler");
    String ext = gzip ? ".ndjson.gz" : ".ndjson";
    return base.resolve("profiler-" + ts + ext);
  }

  static String escape(String s) {
    if (s == null) return "";
    StringBuilder b = new StringBuilder(s.length() + 16);
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      if (c == '\\') b.append("\\\\");
      else if (c == '"') b.append("\\\"");
      else if (c == '\n') b.append("\\n");
      else if (c == '\r') b.append("\\r");
      else if (c == '\t') b.append("\\t");
      else b.append(c);
    }
    return b.toString();
  }
}
