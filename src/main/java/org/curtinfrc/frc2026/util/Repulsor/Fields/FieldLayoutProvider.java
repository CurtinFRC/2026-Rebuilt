/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Fields;

import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;

public interface FieldLayoutProvider {
  FieldTracker.GameElement[] build(FieldTracker ft);

  String gameName();

  int gameYear();
}
