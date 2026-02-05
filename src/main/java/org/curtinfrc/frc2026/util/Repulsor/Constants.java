/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldDefinition;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;

public final class Constants {
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double FIELD_LENGTH = 16.540988;
  public static final double FIELD_WIDTH = aprilTagLayout.getFieldWidth();
  public static final FieldDefinition FIELD = new Rebuilt2026();
}

