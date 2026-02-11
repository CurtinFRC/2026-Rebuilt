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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.curtinfrc.frc2026.util.Repulsor.Constants.FIELD_LENGTH;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.CorridorCenterlineRail;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.GatedAttractorObstacle;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public final class Rebuilt2026 implements FieldDefinition {
  private static final double CORNER_CHAMFER = 0;

  private static final double GRID_Z_MIN_M = -0.05;
  private static final double GRID_Z_MAX_M = 0.35;

  private static final double GRID_CELL_M = 0.75;
  private static final double GRID_REGION_HALF_X_M = Constants.FIELD_LENGTH * 0.5;
  private static final double GRID_REGION_HALF_Y_M = Constants.FIELD_WIDTH * 0.5;

  @Override
  public GameElement[] build(FieldTracker ft) {
    var b = new FieldMapBuilder(ft);

    Pose2d blueOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateBluePose();
    Pose2d redOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateRedPose();

    Pose2d blueHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateBluePose();
    Pose2d redHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateRedPose();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueHub.getX(), blueHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redHub.getX(), redHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueOutpost.getX(), blueOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redOutpost.getX(), redOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    double cx = Constants.FIELD_LENGTH * 0.5;
    double cy = Constants.FIELD_WIDTH * 0.5;

    double x0 = cx - GRID_REGION_HALF_X_M;
    double x1 = cx + GRID_REGION_HALF_X_M;
    double y0 = cy - GRID_REGION_HALF_Y_M;
    double y1 = cy + GRID_REGION_HALF_Y_M;

    int nx = (int) Math.floor((x1 - x0) / GRID_CELL_M);
    int ny = (int) Math.floor((y1 - y0) / GRID_CELL_M);

    for (int ix = 0; ix <= nx; ix++) {
      double x = x0 + ix * GRID_CELL_M + GRID_CELL_M * 0.5;
      for (int iy = 0; iy <= ny; iy++) {
        double y = y0 + iy * GRID_CELL_M + GRID_CELL_M * 0.5;

        b.begin()
            .alliance(Alliance.kBlue)
            .capacity(999)
            .pose(new Pose3d(x, y, 0, null))
            .primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
            .filterType("fuel")
            .category(CategorySpec.kCollect)
            .add();

        b.begin()
            .alliance(Alliance.kRed)
            .capacity(999)
            .pose(new Pose3d(x, y, 0, null))
            .primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
            .filterType("fuel")
            .category(CategorySpec.kCollect)
            .add();
      }
    }

    return b.build();
  }

  @Override
  public String gameName() {
    return "REBUILT";
  }

  @Override
  public int gameYear() {
    return 2026;
  }

  @Override
  public List<Obstacle> fieldObstacles() {
    // double maxRangeY = 1;
    // double maxRangeX = 1.2;

    double rectWidth = 0.5929315;
    double rectHeight = 5.711800;
    double rectHalfY = rectHeight * 0.5;

    double leftRectX = (FIELD_LENGTH / 2) - 3.648981; // 2.5 meters 34 degrees 16 ms
    double rightRectX = (FIELD_LENGTH / 2) + 3.648981;
    double rectCy = Constants.FIELD_WIDTH / 2;

    double gapHeight = Math.max(0.0, (Constants.FIELD_WIDTH * 0.5) - rectHalfY);
    double gapTopY = Constants.FIELD_WIDTH - (gapHeight * 0.5);
    double gapBottomY = gapHeight * 0.5;

    double rectStrength = 2.5;
    double rectRangeX = 1.2;
    double rectRangeY = 1.5;

    double biasStrength = 0.3;
    double biasRange = 1.2;

    double bypassStrengthScale = 1.2;
    double bypassRange = 1.4;

    double edgeOffset = 0.06;

    double rectHalfX = rectWidth * 0.5;
    Translation2d[] leftGate =
        new Translation2d[] {
          new Translation2d(leftRectX - rectHalfX, rectCy - rectHalfY),
          new Translation2d(leftRectX + rectHalfX, rectCy - rectHalfY),
          new Translation2d(leftRectX + rectHalfX, rectCy + rectHalfY),
          new Translation2d(leftRectX - rectHalfX, rectCy + rectHalfY)
        };
    Translation2d[] rightGate =
        new Translation2d[] {
          new Translation2d(rightRectX - rectHalfX, rectCy - rectHalfY),
          new Translation2d(rightRectX + rectHalfX, rectCy - rectHalfY),
          new Translation2d(rightRectX + rectHalfX, rectCy + rectHalfY),
          new Translation2d(rightRectX - rectHalfX, rectCy + rectHalfY)
        };

    double leftInsideX = leftRectX + rectHalfX + edgeOffset;
    double rightInsideX = rightRectX - rectHalfX - edgeOffset;

    double sidePullDx = 1.0;

    double leftPullXOut = leftRectX - sidePullDx;
    double rightPullXOut = rightRectX + sidePullDx;

    double sideBiasStrength = biasStrength * 0.50;
    double sideBiasRange = biasRange * 0.60;

    double sideBypassStrengthScale = bypassStrengthScale * 0.85;
    double sideBypassRange = bypassRange * 0.65;

    double leftPullXIn = leftRectX + sidePullDx;

    double rightPullXIn = rightRectX - sidePullDx;

    double railXWindow = 1.2;
    double corridorHalfWidthGuess = Math.max(0.22, gapHeight * 0.45);
    double railStrength = 2.1;
    double railMaxForce = 2.2;

    return List.of(
        new RectangleObstacle(
            new Translation2d(leftRectX, rectCy),
            rectWidth,
            rectHeight,
            rectStrength,
            rectRangeX,
            rectRangeY),
        new RectangleObstacle(
            new Translation2d(rightRectX, rectCy),
            rectWidth,
            rectHeight,
            rectStrength,
            rectRangeX,
            rectRangeY),
        new GatedAttractorObstacle(
            new Translation2d(leftRectX, gapTopY),
            biasStrength,
            biasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftRectX, gapBottomY),
            biasStrength,
            biasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightRectX, gapTopY),
            biasStrength,
            biasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(rightRectX, gapBottomY),
            biasStrength,
            biasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            bypassStrengthScale,
            bypassRange,
            true),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXIn, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXOut, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXIn, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(leftPullXOut, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            leftGate,
            new Translation2d(leftInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXIn, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXOut, gapTopY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapTopY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXIn, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange),
        new GatedAttractorObstacle(
            new Translation2d(rightPullXOut, gapBottomY),
            sideBiasStrength,
            sideBiasRange,
            rightGate,
            new Translation2d(rightInsideX, gapBottomY),
            sideBypassStrengthScale,
            sideBypassRange),
        new CorridorCenterlineRail(
            leftRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            leftRectX, railXWindow, gapBottomY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            rightRectX, railXWindow, gapTopY, corridorHalfWidthGuess, railStrength, railMaxForce),
        new CorridorCenterlineRail(
            rightRectX,
            railXWindow,
            gapBottomY,
            corridorHalfWidthGuess,
            railStrength,
            railMaxForce));
  }

  // @Override // TRENCH + BUMP
  // public List<Obstacle> fieldObstacles() {
  //   double maxRangeY = 0.9;
  //   double maxRangeX = 0.5;
  //   double strength = 1.4;
  //   return List.of(
  //       new SquareObstacle(new Translation2d(4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 1,
  // 1.4),
  //       new SquareObstacle(
  //           new Translation2d(FIELD_LENGTH - 4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 1,
  // 1.4),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) - 3.63982, 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) - 3.63982, Constants.FIELD_WIDTH - 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) + 3.63982, 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY),
  //       new RectangleObstacle(
  //           new Translation2d((FIELD_LENGTH / 2) + 3.63982, Constants.FIELD_WIDTH - 1.4224),
  //           1.1938,
  //           0.255336,
  //           strength,
  //           maxRangeX,
  //           maxRangeY));
  // }

  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 1, true),
        new HorizontalObstacle(Constants.FIELD_WIDTH, 1, false),
        new VerticalObstacle(0.0, 2, true),
        new VerticalObstacle(Constants.FIELD_LENGTH, 2, false));
  }

  @Override
  public Heatmap getHeatmap() {
    final double SLOW_HEAT = 0.6;
    final double FAST_HEAT = 1.0;

    final double TRANS_M = 0.30;

    double cx = Constants.FIELD_LENGTH * 0.5;
    double cy = Constants.FIELD_WIDTH * 0.5;

    double x0 = cx - GRID_REGION_HALF_X_M;
    double x1 = cx + GRID_REGION_HALF_X_M;
    double y0 = cy - GRID_REGION_HALF_Y_M;
    double y1 = cy + GRID_REGION_HALF_Y_M;

    double half = 1.1938 * 0.5;

    double leftSqCx = 4.625594;
    double leftRectCx = (FIELD_LENGTH * 0.5) - 3.63982;

    double rightSqCx = FIELD_LENGTH - 4.625594;
    double rightRectCx = (FIELD_LENGTH * 0.5) + 3.63982;

    double leftBandX0 = Math.min(leftSqCx, leftRectCx) - half;
    double leftBandX1 = Math.max(leftSqCx, leftRectCx) + half;

    double rightBandX0 = Math.min(rightSqCx, rightRectCx) - half;
    double rightBandX1 = Math.max(rightSqCx, rightRectCx) + half;

    leftBandX0 = MathUtil.clamp(leftBandX0, x0, x1);
    leftBandX1 = MathUtil.clamp(leftBandX1, x0, x1);
    rightBandX0 = MathUtil.clamp(rightBandX0, x0, x1);
    rightBandX1 = MathUtil.clamp(rightBandX1, x0, x1);

    if (leftBandX1 > rightBandX0) {
      double mid = 0.5 * (leftBandX1 + rightBandX0);
      leftBandX1 = mid;
      rightBandX0 = mid;
    }

    double fastL = Math.max(0.0, leftBandX0 - x0);
    double fastM = Math.max(0.0, rightBandX0 - leftBandX1);
    double fastR = Math.max(0.0, x1 - rightBandX1);

    double eL = MathUtil.clamp(TRANS_M, 0.0, fastL);
    double eML = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
    double eMR = MathUtil.clamp(TRANS_M, 0.0, fastM * 0.5);
    double eR = MathUtil.clamp(TRANS_M, 0.0, fastR);

    double fastLCoreW = Math.max(0.0, fastL - eL);
    double fastMCoreW = Math.max(0.0, fastM - eML - eMR);
    double fastRCoreW = Math.max(0.0, fastR - eR);

    var hb = Heatmap.builder();

    double curX = x0;

    if (fastLCoreW > 0.0) {
      hb.block("FAST_L_CORE", new Translation2d(curX, y0), fastLCoreW, y1 - y0, FAST_HEAT);
      curX += fastLCoreW;
    }
    if (eL > 0.0) {
      hb.block("FAST_L_EDGE", new Translation2d(curX, y0), eL, y1 - y0, FAST_HEAT);
      curX += eL;
    }

    hb.block(
        "SLOW_L", new Translation2d(leftBandX0, y0), leftBandX1 - leftBandX0, y1 - y0, SLOW_HEAT);
    curX = leftBandX1;

    if (eML > 0.0) {
      hb.block("FAST_M_L_EDGE", new Translation2d(curX, y0), eML, y1 - y0, FAST_HEAT);
      curX += eML;
    }
    if (fastMCoreW > 0.0) {
      hb.block("FAST_M_CORE", new Translation2d(curX, y0), fastMCoreW, y1 - y0, FAST_HEAT);
      curX += fastMCoreW;
    }
    if (eMR > 0.0) {
      hb.block("FAST_M_R_EDGE", new Translation2d(curX, y0), eMR, y1 - y0, FAST_HEAT);
      curX += eMR;
    }

    hb.block(
        "SLOW_R",
        new Translation2d(rightBandX0, y0),
        rightBandX1 - rightBandX0,
        y1 - y0,
        SLOW_HEAT);
    curX = rightBandX1;

    if (eR > 0.0) {
      hb.block("FAST_R_EDGE", new Translation2d(curX, y0), eR, y1 - y0, FAST_HEAT);
      curX += eR;
    }
    if (fastRCoreW > 0.0) {
      hb.block("FAST_R_CORE", new Translation2d(curX, y0), fastRCoreW, y1 - y0, FAST_HEAT);
      curX += fastRCoreW;
    }

    if (eL > 0.0) hb.transition("SLOW_L", "FAST_L_EDGE", 1.0);
    if (eML > 0.0) hb.transition("SLOW_L", "FAST_M_L_EDGE", 1.0);

    if (eMR > 0.0) hb.transition("SLOW_R", "FAST_M_R_EDGE", 1.0);
    if (eR > 0.0) hb.transition("SLOW_R", "FAST_R_EDGE", 1.0);

    return hb.build();
  }
}


