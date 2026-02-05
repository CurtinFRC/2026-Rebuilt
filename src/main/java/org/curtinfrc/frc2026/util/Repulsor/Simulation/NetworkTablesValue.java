/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Simulation;

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

