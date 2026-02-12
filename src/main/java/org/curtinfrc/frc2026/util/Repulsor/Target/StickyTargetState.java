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

final class StickyTargetState<T> {
  T lastOut;
  double lastOutChangeSec = -1e9;

  double stickyInvalidSinceSec = -1e9;

  T bestValidKey;
  double bestMissingSinceSec = -1e9;
  double bestValidSinceSec = -1e9;

  T lastBestSeen;
  double bestLastChangeSec = -1e9;
  double flickerSinceSec = -1e9;

  T sticky;
  double stickySinceSec = -1e9;
  double stickyLastBestSeenSec = -1e9;

  T candidate;
  double candidateSinceSec = -1e9;

  T lastSticky;
  double lastSwitchSec = -1e9;

  double lastUpdateSec = 0.0;

  void clear() {
    lastOut = null;
    lastOutChangeSec = -1e9;

    stickyInvalidSinceSec = -1e9;

    bestValidKey = null;
    bestMissingSinceSec = -1e9;
    bestValidSinceSec = -1e9;

    lastBestSeen = null;
    bestLastChangeSec = -1e9;
    flickerSinceSec = -1e9;

    sticky = null;
    stickySinceSec = -1e9;
    stickyLastBestSeenSec = -1e9;

    candidate = null;
    candidateSinceSec = -1e9;

    lastSticky = null;
    lastSwitchSec = -1e9;

    lastUpdateSec = 0.0;
  }
}
