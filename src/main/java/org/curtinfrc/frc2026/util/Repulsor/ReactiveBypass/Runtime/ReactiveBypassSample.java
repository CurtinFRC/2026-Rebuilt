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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Translation2d;

final class ReactiveBypassSample {
  final Translation2d pos;
  final double sPara;
  final double sPerp;
  final int signPara;
  final int signPerp;
  final double dt;

  ReactiveBypassSample(
      Translation2d pos, double sPara, double sPerp, int signPara, int signPerp, double dt) {
    this.pos = pos;
    this.sPara = sPara;
    this.sPerp = sPerp;
    this.signPara = signPara;
    this.signPerp = signPerp;
    this.dt = dt;
  }
}
