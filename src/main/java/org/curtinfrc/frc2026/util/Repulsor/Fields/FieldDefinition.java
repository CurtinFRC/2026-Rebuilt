package org.curtinfrc.frc2026.util.Repulsor.Fields;

import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap.HeatmapProvider;

public interface FieldDefinition
    extends FieldLayoutProvider, FieldPlanner.ObstacleProvider, HeatmapProvider {}
