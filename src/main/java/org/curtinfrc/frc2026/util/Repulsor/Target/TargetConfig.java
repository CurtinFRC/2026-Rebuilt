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
  public static final double FORCE_SWITCH_STILL_SEC = 0.10;
  public static final double FORCE_SWITCH_FLICKER_SEC = 0.20;

  public static final double STICKY_INVALID_DEBOUNCE_SEC = 0.35;

  public static final double BASE_HARD_LOCK_SEC = 0.25;
  public static final double HARD_LOCK_PER_M_SEC = 0.18;
  public static final double HARD_LOCK_MAX_SEC = 1.20;

  public static final double EARLY_SWITCH_MULT = 1.25;
  public static final double SWITCH_BACK_EPS_MIN = 0.35;

  public static final double PINGPONG_EMA_ALPHA = 0.35;

  public static final double PINGPONG_WINDOW_MIN_SEC = 0.50;
  public static final double PINGPONG_WINDOW_MAX_SEC = 2.10;

  public static final double PINGPONG_BLOCK_MIN_SEC = 0.60;
  public static final double PINGPONG_BLOCK_MAX_SEC = 2.20;

  public static final double PINGPONG_WINDOW_BASE_SEC = 0.60;
  public static final double PINGPONG_WINDOW_PER_M_SEC = 0.45;

  public static final double PINGPONG_BLOCK_BASE_SEC = 0.70;
  public static final double PINGPONG_BLOCK_PER_M_SEC = 0.45;

  public static final double MIN_SWITCH_STABLE_SEC = 0.05;
  public static final double FLICKER_DELTA_SEC = 0.10;
  public static final double FLICKER_RESET_STABLE_SEC = 0.18;

  public static final double BEST_VALID_DEBOUNCE_SEC = 0.06;
  public static final double BEST_MISSING_DEBOUNCE_SEC = 0.10;
  public static final double BEST_MISSING_DROP_SEC = 0.40;

  public static final double DEFAULT_SEEN_TIMEOUT_SEC = 0.45;

  public static final double CANON_EVICT_SEC = 5.0;
  public static final int CANON_MAX = 28;

  public static final double FLICKER_FREEZE_EXTEND_SEC = 0.30;
  public static final double HARD_LOCK_OVERRIDE_MULT = 2.00;

  public static final double CLOSE_PAIR_DIST_ADD = 0.15;
  public static final double CLOSE_PAIR_REQ_MULT = 1.15;
}
