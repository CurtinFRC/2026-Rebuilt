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
package org.curtinfrc.frc2026.util.Repulsor.Target;

public class TargetConfig {
  /*
   * Time the candidate target must stay unchanged before forcing a switch.
   */
  public static final double FORCE_SWITCH_STILL_SEC = 0.10;
  /*
   * Extended switch-forcing window used when target identity is flickering.
   */
  public static final double FORCE_SWITCH_FLICKER_SEC = 0.20;

  /*
   * Debounce duration for treating temporarily invalid targets as sticky.
   */
  public static final double STICKY_INVALID_DEBOUNCE_SEC = 0.25;

  /*
   * Base hard-lock time that prevents immediate retargeting after a choice.
   */
  public static final double BASE_HARD_LOCK_SEC = 0.25;
  /*
   * Additional hard-lock time added per meter of target distance.
   */
  public static final double HARD_LOCK_PER_M_SEC = 0.1;
  /*
   * Upper bound for any computed hard-lock hold duration.
   */
  public static final double HARD_LOCK_MAX_SEC = 1.20;

  /*
   * Multiplier for early switching confidence while evaluating alternatives.
   */
  public static final double EARLY_SWITCH_MULT = 1.55;
  /*
   * Minimum score gap needed before allowing a switch back to a prior target.
   */
  public static final double SWITCH_BACK_EPS_MIN = 0.35;

  /*
   * Smoothing factor for EMA-based ping-pong detection between targets.
   */
  public static final double PINGPONG_EMA_ALPHA = 0.35;

  /*
   * Lower clamp for the ping-pong observation time window.
   */
  public static final double PINGPONG_WINDOW_MIN_SEC = 0.50;
  /*
   * Upper clamp for the ping-pong observation time window.
   */
  public static final double PINGPONG_WINDOW_MAX_SEC = 2.10;

  /*
   * Lower clamp for how long ping-pong switching is blocked.
   */
  public static final double PINGPONG_BLOCK_MIN_SEC = 0.60;
  /*
   * Upper clamp for how long ping-pong switching is blocked.
   */
  public static final double PINGPONG_BLOCK_MAX_SEC = 2.20;

  /*
   * Base duration for ping-pong detection window sizing.
   */
  public static final double PINGPONG_WINDOW_BASE_SEC = 0.60;
  /*
   * Additional ping-pong window duration added per meter of distance.
   */
  public static final double PINGPONG_WINDOW_PER_M_SEC = 0.45;

  /*
   * Base duration for blocking switches after ping-pong is detected.
   */
  public static final double PINGPONG_BLOCK_BASE_SEC = 0.70;
  /*
   * Additional ping-pong block duration added per meter of distance.
   */
  public static final double PINGPONG_BLOCK_PER_M_SEC = 0.45;

  /*
   * Minimum stable time required before accepting a candidate switch.
   */
  public static final double MIN_SWITCH_STABLE_SEC = 0.05;
  /*
   * Time-delta threshold used to classify oscillation as flicker.
   */
  public static final double FLICKER_DELTA_SEC = 0.10;
  /*
   * Stable period needed to clear active flicker state.
   */
  public static final double FLICKER_RESET_STABLE_SEC = 0.18;

  /*
   * Debounce for promoting a "best" target when it is consistently valid.
   */
  public static final double BEST_VALID_DEBOUNCE_SEC = 0.06;
  /*
   * Debounce for tolerating short "best target missing" gaps.
   */
  public static final double BEST_MISSING_DEBOUNCE_SEC = 0.10;
  /*
   * Maximum missing duration before dropping the current best target.
   */
  public static final double BEST_MISSING_DROP_SEC = 0.40;

  /*
   * Default timeout before a target is considered no longer seen.
   */
  public static final double DEFAULT_SEEN_TIMEOUT_SEC = 0.45;

  /*
   * Age threshold for evicting stale entries from canonical target history.
   */
  public static final double CANON_EVICT_SEC = 5.0;
  /*
   * Maximum number of canonical target entries retained.
   */
  public static final int CANON_MAX = 28;

  /*
   * Additional freeze time applied when flicker conditions are active.
   */
  public static final double FLICKER_FREEZE_EXTEND_SEC = 0.30;
  /*
   * Multiplier that extends hard-lock time during override conditions.
   */
  public static final double HARD_LOCK_OVERRIDE_MULT = 2.00;

  /*
   * Extra distance margin used to identify close target pairs.
   */
  public static final double CLOSE_PAIR_DIST_ADD = 0.15;
  /*
   * Required confidence multiplier when comparing close target pairs.
   */
  public static final double CLOSE_PAIR_REQ_MULT = 1.15;
}
