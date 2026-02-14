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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;

public final class DefenceBehaviour extends Behaviour {
  private final int prio;
  private final Supplier<RepulsorSetpoint> defenseGoal;
  private final Supplier<Double> speedCap;

  public DefenceBehaviour(
      int priority, Supplier<RepulsorSetpoint> defenseGoal, Supplier<Double> speedCap) {
    this.prio = priority;
    this.defenseGoal = defenseGoal;
    this.speedCap = speedCap;
  }

  @Override
  public String name() {
    return "Defense";
  }

  @Override
  public int priority() {
    return prio;
  }

  @Override
  public boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    return flags.contains(BehaviourFlag.DEFENCE_MODE);
  }

  private static SetpointContext makeCtx(BehaviourContext ctx, Pose2d robotPose) {
    return new SetpointContext(
        Optional.ofNullable(robotPose),
        Math.max(0.0, ctx.robot_x) * 2.0,
        Math.max(0.0, ctx.robot_y) * 2.0,
        0.0,
        ctx.vision.getObstacles());
  }

  @Override
  public Command build(BehaviourContext ctx) {
    return Commands.run(
            () -> {
              Pose2d robotPose = ctx.robotPose.get();
              RepulsorSetpoint sp = defenseGoal != null ? defenseGoal.get() : null;
              if (sp == null) {
                ctx.drive.runVelocity(new ChassisSpeeds());
                return;
              }

              Pose2d goalPose = sp.get(makeCtx(ctx, robotPose));
              ctx.repulsor.setCurrentGoal(sp);
              ctx.planner.setRequestedGoal(goalPose);

              RepulsorSample sample =
                  ctx.planner.calculate(
                      robotPose,
                      ctx.vision.getObstacles(),
                      ctx.robot_x,
                      ctx.robot_y,
                      CategorySpec.kEndgame,
                      false,
                      0.0);

              ChassisSpeeds speeds =
                  sample.asChassisSpeeds(
                      ctx.repulsor.getDrive().getOmegaPID(), robotPose.getRotation());

              double cap = speedCap != null ? Math.max(0.25, speedCap.get()) : 2.8;
              speeds.vxMetersPerSecond = Math.max(-cap, Math.min(cap, speeds.vxMetersPerSecond));
              speeds.vyMetersPerSecond = Math.max(-cap, Math.min(cap, speeds.vyMetersPerSecond));

              ctx.drive.runVelocity(speeds);
            },
            ctx.drive)
        .finallyDo(i -> ctx.drive.runVelocity(new ChassisSpeeds()));
  }
}
