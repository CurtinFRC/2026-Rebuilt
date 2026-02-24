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

package org.curtinfrc.frc2026.util.Repulsor.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import java.util.Objects;

public final class NetworkTablesValue<T> {
  public interface Codec<T> {
    Class<T> type();

    void publish(NetworkTableInstance inst, String topicName, T initialValue);

    void close();

    T get();

    void set(T value);

    void flush();
  }

  private final Codec<T> codec;

  public static String toAdvantageKit(String topicName) {
    return "AdvantageKit/RealOutputs" + (topicName.startsWith("/") ? "" : "/") + topicName;
  }

  public NetworkTablesValue(
      Codec<T> codec, NetworkTableInstance inst, String topicName, T initialValue) {
    this.codec = Objects.requireNonNull(codec, "codec");
    Objects.requireNonNull(inst, "inst");
    if (topicName == null || topicName.isEmpty()) throw new IllegalArgumentException("topicName");
    codec.publish(inst, topicName, initialValue);
  }

  public T get() {
    return codec.get();
  }

  public void set(T value) {
    codec.set(value);
  }

  public void flush() {
    codec.flush();
  }

  public void close() {
    codec.close();
  }

  public static NetworkTablesValue<Double> ofDouble(
      NetworkTableInstance inst, String topicName, double initialValue) {
    return new NetworkTablesValue<>(new DoubleCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<Long> ofInteger(
      NetworkTableInstance inst, String topicName, long initialValue) {
    return new NetworkTablesValue<>(new IntegerCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<Boolean> ofBoolean(
      NetworkTableInstance inst, String topicName, boolean initialValue) {
    return new NetworkTablesValue<>(new BooleanCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<String> ofString(
      NetworkTableInstance inst, String topicName, String initialValue) {
    return new NetworkTablesValue<>(new StringCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<double[]> ofDoubleArray(
      NetworkTableInstance inst, String topicName, double[] initialValue) {
    return new NetworkTablesValue<>(new DoubleArrayCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<Pose2d> ofPose2d(
      NetworkTableInstance inst, String topicName, Pose2d initialValue) {
    return new NetworkTablesValue<>(new Pose2dCodec(), inst, topicName, initialValue);
  }

  public static NetworkTablesValue<Translation2d> ofTranslation2d(
      NetworkTableInstance inst, String topicName, Translation2d initialValue) {
    return new NetworkTablesValue<>(new Translation2dCodec(), inst, topicName, initialValue);
  }

  private static final class DoubleCodec implements Codec<Double> {
    private DoublePublisher pub;
    private DoubleSubscriber sub;

    @Override
    public Class<Double> type() {
      return Double.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, Double initialValue) {
      DoubleTopic topic = inst.getDoubleTopic(topicName);
      pub = topic.publish();
      double init = initialValue != null ? initialValue : 0.0;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedDouble existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    @Override
    public Double get() {
      return sub.get();
    }

    @Override
    public void set(Double value) {
      pub.set(value != null ? value : 0.0);
    }

    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class IntegerCodec implements Codec<Long> {
    private IntegerPublisher pub;
    private IntegerSubscriber sub;

    @Override
    public Class<Long> type() {
      return Long.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, Long initialValue) {
      IntegerTopic topic = inst.getIntegerTopic(topicName);
      pub = topic.publish();
      long init = initialValue != null ? initialValue : 0L;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedInteger existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    @Override
    public Long get() {
      return sub.get();
    }

    @Override
    public void set(Long value) {
      pub.set(value != null ? value : 0L);
    }

    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class BooleanCodec implements Codec<Boolean> {
    private BooleanPublisher pub;
    private BooleanSubscriber sub;

    @Override
    public Class<Boolean> type() {
      return Boolean.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, Boolean initialValue) {
      BooleanTopic topic = inst.getBooleanTopic(topicName);
      pub = topic.publish();
      boolean init = initialValue != null ? initialValue : false;
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedBoolean existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    @Override
    public Boolean get() {
      return sub.get();
    }

    @Override
    public void set(Boolean value) {
      pub.set(value != null ? value : false);
    }

    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class StringCodec implements Codec<String> {
    private StringPublisher pub;
    private StringSubscriber sub;

    @Override
    public Class<String> type() {
      return String.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, String initialValue) {
      StringTopic topic = inst.getStringTopic(topicName);
      pub = topic.publish();
      String init = initialValue != null ? initialValue : "";
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedString existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    @Override
    public String get() {
      return sub.get();
    }

    @Override
    public void set(String value) {
      pub.set(value != null ? value : "");
    }

    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }

  private static final class Pose2dCodec implements Codec<Pose2d> {
    private StringPublisher x;
    private StringPublisher y;
    private StringPublisher theta;
    private StringSubscriber xSub;
    private StringSubscriber ySub;
    private StringSubscriber thetaSub;

    public Pose2dCodec() {
      // No-op
    }

    @Override
    public Class<Pose2d> type() {
      return Pose2d.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, Pose2d initialValue) {
      x = inst.getStringTopic(topicName + "/x").publish();
      y = inst.getStringTopic(topicName + "/y").publish();
      theta = inst.getStringTopic(topicName + "/theta").publish();
      xSub = inst.getStringTopic(topicName + "/x").subscribe("0.0");
      ySub = inst.getStringTopic(topicName + "/y").subscribe("0.0");
      thetaSub = inst.getStringTopic(topicName + "/theta").subscribe("0.0");
      Pose2d init = initialValue != null ? initialValue : new Pose2d();
      set(init);
    }

    @Override
    public Pose2d get() {
      double xVal = Double.parseDouble(xSub.get());
      double yVal = Double.parseDouble(ySub.get());
      double thetaVal = Double.parseDouble(thetaSub.get());
      return new Pose2d(xVal, yVal, new Rotation2d(thetaVal));
    }

    @Override
    public void set(Pose2d value) {
      Pose2d val = value != null ? value : new Pose2d();
      x.set(Double.toString(val.getX()));
      y.set(Double.toString(val.getY()));
      theta.set(Double.toString(val.getRotation().getRadians()));
    }

    @Override
    public void flush() {
      if (x != null) x.getTopic().getInstance().flush();
      if (y != null) y.getTopic().getInstance().flush();
      if (theta != null) theta.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (xSub != null) xSub.close();
      if (ySub != null) ySub.close();
      if (thetaSub != null) thetaSub.close();
      if (x != null) x.close();
      if (y != null) y.close();
      if (theta != null) theta.close();
      xSub = null;
      ySub = null;
      thetaSub = null;
      x = null;
      y = null;
      theta = null;
    }
  }

  private static final class Translation2dCodec implements Codec<Translation2d> {
    private StringPublisher x;
    private StringPublisher y;
    private StringSubscriber xSub;
    private StringSubscriber ySub;

    public Translation2dCodec() {
      // No-op
    }

    @Override
    public Class<Translation2d> type() {
      return Translation2d.class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, Translation2d initialValue) {
      x = inst.getStringTopic(topicName + "/x").publish();
      y = inst.getStringTopic(topicName + "/y").publish();
      xSub = inst.getStringTopic(topicName + "/x").subscribe("0.0");
      ySub = inst.getStringTopic(topicName + "/y").subscribe("0.0");
      Translation2d init = initialValue != null ? initialValue : new Translation2d();
      set(init);
    }

    @Override
    public Translation2d get() {
      double xVal = Double.parseDouble(xSub.get());
      double yVal = Double.parseDouble(ySub.get());
      return new Translation2d(xVal, yVal);
    }

    @Override
    public void set(Translation2d value) {
      Translation2d val = value != null ? value : new Translation2d();
      x.set(Double.toString(val.getX()));
      y.set(Double.toString(val.getY()));
    }

    @Override
    public void flush() {
      if (x != null) x.getTopic().getInstance().flush();
      if (y != null) y.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (xSub != null) xSub.close();
      if (ySub != null) ySub.close();
      if (x != null) x.close();
      if (y != null) y.close();
      xSub = null;
      ySub = null;
      x = null;
      y = null;
    }
  }

  private static final class DoubleArrayCodec implements Codec<double[]> {
    private DoubleArrayPublisher pub;
    private DoubleArraySubscriber sub;

    @Override
    public Class<double[]> type() {
      return double[].class;
    }

    @Override
    public void publish(NetworkTableInstance inst, String topicName, double[] initialValue) {
      DoubleArrayTopic topic = inst.getDoubleArrayTopic(topicName);
      pub = topic.publish();
      double[] init = initialValue != null ? initialValue : new double[0];
      sub = topic.subscribe(init);
      edu.wpi.first.networktables.TimestampedDoubleArray existing = sub.getAtomic();
      if (existing == null || existing.timestamp == 0) {
        pub.set(init);
      }
    }

    @Override
    public double[] get() {
      return sub.get();
    }

    @Override
    public void set(double[] value) {
      pub.set(value != null ? value : new double[0]);
    }

    @Override
    public void flush() {
      if (pub != null) pub.getTopic().getInstance().flush();
    }

    @Override
    public void close() {
      if (sub != null) sub.close();
      if (pub != null) pub.close();
      sub = null;
      pub = null;
    }
  }
}
