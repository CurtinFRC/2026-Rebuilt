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

import java.util.Comparator;
import java.util.Objects;

public final class Interval<T> {
  public enum BoundType {
    OPEN,
    CLOSED
  }

  private final T first;
  private final T second;
  private final Comparator<? super T> cmp;
  private final BoundType lowerBound;
  private final BoundType upperBound;

  private Interval(
      T first, T second, Comparator<? super T> cmp, BoundType lowerBound, BoundType upperBound) {
    this.first = Objects.requireNonNull(first, "first");
    this.second = Objects.requireNonNull(second, "second");
    this.cmp = Objects.requireNonNull(cmp, "cmp");
    this.lowerBound = Objects.requireNonNull(lowerBound, "lowerBound");
    this.upperBound = Objects.requireNonNull(upperBound, "upperBound");
  }

  public static <T extends Comparable<? super T>> Interval<T> closed(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.CLOSED, BoundType.CLOSED);
  }

  public static <T extends Comparable<? super T>> Interval<T> open(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.OPEN, BoundType.OPEN);
  }

  public static <T extends Comparable<? super T>> Interval<T> closedOpen(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.CLOSED, BoundType.OPEN);
  }

  public static <T extends Comparable<? super T>> Interval<T> openClosed(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.OPEN, BoundType.CLOSED);
  }

  public static <T> Interval<T> of(
      T a, T b, Comparator<? super T> comparator, BoundType lowerBound, BoundType upperBound) {
    return new Interval<>(a, b, comparator, lowerBound, upperBound);
  }

  public T first() {
    return first;
  }

  public T second() {
    return second;
  }

  public T min() {
    return cmp.compare(first, second) <= 0 ? first : second;
  }

  public T max() {
    return cmp.compare(first, second) <= 0 ? second : first;
  }

  public boolean isEmpty() {
    int c = cmp.compare(first, second);
    if (c == 0) return lowerBound == BoundType.OPEN || upperBound == BoundType.OPEN;
    return false;
  }

  public boolean within(T x) {
    Objects.requireNonNull(x, "x");

    T lo = min();
    T hi = max();

    int cl = cmp.compare(x, lo);
    int ch = cmp.compare(x, hi);

    boolean okLower = (lowerBound == BoundType.CLOSED) ? (cl >= 0) : (cl > 0);
    boolean okUpper = (upperBound == BoundType.CLOSED) ? (ch <= 0) : (ch < 0);

    return okLower && okUpper;
  }

  public boolean contains(T x) {
    return within(x);
  }

  public boolean overlaps(Interval<T> other) {
    Objects.requireNonNull(other, "other");

    if (this.isEmpty() || other.isEmpty()) return false;

    T aLo = this.min(), aHi = this.max();
    T bLo = other.min(), bHi = other.max();

    int left = cmp.compare(aHi, bLo);
    int right = cmp.compare(bHi, aLo);

    if (left < 0 || right < 0) return false;

    if (left == 0)
      return this.upperBound == BoundType.CLOSED && other.lowerBound == BoundType.CLOSED;
    if (right == 0)
      return other.upperBound == BoundType.CLOSED && this.lowerBound == BoundType.CLOSED;

    return true;
  }
}

