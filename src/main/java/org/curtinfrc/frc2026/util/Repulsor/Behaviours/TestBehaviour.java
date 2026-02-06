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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;

public final class TestBehaviour extends Behaviour {
  private final NetworkTablesValue<Double> shotAngle =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotAngle"), 0.0);

  private final NetworkTablesValue<Double> shotSpeed =
      NetworkTablesValue.ofDouble(
          NetworkTableInstance.getDefault(), NetworkTablesValue.toAdvantageKit("/ShotSpeed"), 0.0);

  public TestBehaviour() {}

  @Override
  public String name() {
    return "Test";
  }

  @Override
  public int priority() {
    return 1000;
  }

  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.TEST_MODE);
  }

  private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
    return new SetpointContext(
        Optional.ofNullable(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        ctx.coral_offset,
        ctx.algae_offset,
        0.0,
        ctx.vision.getObstacles());
  }

  @Override
  public Command build(BehaviourContext ctx) {
    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();

              RepulsorSetpoint sp =
                  new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET);
              Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));

              ctx.repulsor.setCurrentGoal(sp);
              ctx.planner.setRequestedGoal(goalPose);

              Setpoints.Rebuilt2026.getHubShotSolution(makeCtx(ctx, robotPose))
                  .ifPresent(
                      sol -> {
                        shotSpeed.set(sol.launchSpeedMetersPerSecond());
                        shotAngle.set(sol.launchAngle().getDegrees());
                      });

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      ctx.coral_offset,
                      ctx.algae_offset,
                      CategorySpec.kCollect,
                      false,
                      0.0);

              ChassisSpeeds speeds =
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation());

              ctx.drive.runVelocity(speeds);
            },
            ctx.drive)
        .finallyDo(i -> ctx.drive.runVelocity(new ChassisSpeeds()));
  }
}
