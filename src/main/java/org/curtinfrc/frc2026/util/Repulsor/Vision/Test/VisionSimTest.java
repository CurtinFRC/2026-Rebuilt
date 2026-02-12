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

package org.curtinfrc.frc2026.util.Repulsor.Vision.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import java.util.Random;
import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Vision.RepulsorVision;

public class VisionSimTest implements RepulsorVision {
  private static final boolean USE_CONTROLLER = true;
  private static final boolean USE_KEYBOARD = true;
  private static final int ROBOT_COUNT = 6;
  private static final double STUCK_SPEED_THRESH = 0.08;
  private static final double STUCK_TIME = 1.0;

  private final Random rng;
  private final VisionSimWorld.Agent[] agents;
  private final AgentController controller;
  private final ArrayList<Pose2d>[] poseHistory;
  private final double[] stuckTimer;
  private final double[] lastX;
  private final double[] lastY;
  private Pose2d rPose = new Pose2d();
  private final CommandXboxController driver = new CommandXboxController(2);
  private final Joystick keyboard = new Joystick(3);
  private double lastTime;
  private boolean timeInitialized;

  @SuppressWarnings("unchecked")
  public VisionSimTest() {
    ArrayList<Setpoints.GameSetpoint> scoreTargets = new ArrayList<>();
    scoreTargets.add(Setpoints.Rebuilt2026.HUB_SHOOT);

    if (USE_CONTROLLER) {
      rng = null;
      agents = null;
      controller = null;
      poseHistory = null;
      stuckTimer = null;
      lastX = null;
      lastY = null;
    } else {
      rng = new Random(System.currentTimeMillis() ^ Double.doubleToLongBits(Math.random()));
      agents = new VisionSimWorld.Agent[ROBOT_COUNT];
      controller = new AgentController(rng, scoreTargets);
      poseHistory = new ArrayList[ROBOT_COUNT];
      stuckTimer = new double[ROBOT_COUNT];
      lastX = new double[ROBOT_COUNT];
      lastY = new double[ROBOT_COUNT];

      int nScorers = 1;
      int nBlockers = Math.max(2, ROBOT_COUNT - 2);
      int nRoamers = Math.max(0, ROBOT_COUNT - nScorers - nBlockers);

      ArrayList<AgentRole> roles = new ArrayList<>();
      for (int i = 0; i < nScorers; i++) {
        roles.add(AgentRole.SCORER);
      }
      for (int i = 0; i < nBlockers; i++) {
        roles.add(AgentRole.BLOCKER);
      }
      for (int i = 0; i < nRoamers; i++) {
        roles.add(AgentRole.ROAMER);
      }
      Collections.shuffle(roles, rng);

      for (int i = 0; i < ROBOT_COUNT; i++) {
        int team = (i < ROBOT_COUNT / 2) ? 0 : 1;
        double rx = 0.7 + 0.3 * rng.nextDouble();
        double ry = 0.7 + 0.3 * rng.nextDouble();
        agents[i] = new VisionSimWorld.Agent(rng, roles.get(i), team, rx, ry);
        poseHistory[i] = new ArrayList<>();
        stuckTimer[i] = 0.0;
        lastX[i] = agents[i].x;
        lastY[i] = agents[i].y;
      }
    }

    lastTime = 0.0;
    timeInitialized = false;
  }

  private void resetAgent(int index) {
    VisionSimWorld.Agent old = agents[index];
    VisionSimWorld.Agent repl =
        new VisionSimWorld.Agent(rng, old.role, old.team, old.rx, old.ry)
            .withCaps(old.maxSpeed, old.maxAccel);
    agents[index] = repl;
    poseHistory[index].clear();
    stuckTimer[index] = 0.0;
    lastX[index] = repl.x;
    lastY[index] = repl.y;
  }

  @Override
  public Obstacle[] getObstacles() {
    if (!USE_CONTROLLER) {
      Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isEmpty()) {
        alliance = Optional.of(Alliance.Red);
      }

      Obstacle[] out = new Obstacle[ROBOT_COUNT];
      for (int i = 0; i < ROBOT_COUNT; i++) {
        VisionSimWorld.Agent a = agents[i];
        Pose2d pose = new Pose2d(a.x, a.y, Rotation2d.fromRadians(a.heading));
        RepulsorVision.Kind k =
            alliance.get() == Alliance.Blue
                ? RepulsorVision.Kind.kRobotRed
                : RepulsorVision.Kind.kRobotBlue;

        out[i] = new RepulsorVision.Obstacle(pose, new RepulsorVision.ObstacleType(a.rx, a.ry, k));

        // Logger.recordOutput("VisionSim/Agent" + i + "/Pose", pose);
      }
      return out;
    }

    Obstacle[] out = new Obstacle[1];
    out[0] =
        new RepulsorVision.Obstacle(
            rPose,
            new RepulsorVision.ObstacleType(
                0.8,
                0.8,
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? RepulsorVision.Kind.kRobotRed
                    : RepulsorVision.Kind.kRobotBlue));
    return out;
  }

  @Override
  public void tick() {
    double now = Timer.getFPGATimestamp();
    if (!timeInitialized) {
      lastTime = now;
      timeInitialized = true;
      return;
    }
    double dt = now - lastTime;
    lastTime = now;
    dt = Math.min(VisionSimWorld.DT_MAX, Math.max(VisionSimWorld.DT_MIN, dt));

    if (!USE_CONTROLLER) {
      if (agents == null) {
        return;
      }
      for (int i = 0; i < ROBOT_COUNT; i++) {
        VisionSimWorld.Agent a = agents[i];
        double prevX = a.x;
        double prevY = a.y;
        double prevHeading = a.heading;

        Translation2d tgt = controller.nextTarget(a, dt);
        controller.step(a, tgt, dt);

        Pose2d prevPose = new Pose2d(prevX, prevY, Rotation2d.fromRadians(prevHeading));
        Pose2d candidate = new Pose2d(a.x, a.y, Rotation2d.fromRadians(a.heading));
        Pose2d safePose = enforceSelfClearance(prevPose, candidate);

        a.x = safePose.getX();
        a.y = safePose.getY();
        a.heading = safePose.getRotation().getRadians();
        poseHistory[i].add(safePose);

        double disp = Math.hypot(a.x - lastX[i], a.y - lastY[i]);
        double speed = disp / dt;
        if (speed < STUCK_SPEED_THRESH) {
          stuckTimer[i] += dt;
        } else {
          stuckTimer[i] = 0.0;
        }
        if (stuckTimer[i] > STUCK_TIME) {
          resetAgent(i);
          a = agents[i];
        }
        lastX[i] = a.x;
        lastY[i] = a.y;
      }
      return;
    }

    double fwd;
    double strafe;
    double rot;
    if (USE_KEYBOARD) {
      fwd = -keyboard.getRawAxis(0);
      strafe = keyboard.getRawAxis(1);
      rot = keyboard.getRawAxis(2);
    } else {
      fwd = driver.getLeftX();
      strafe = -driver.getLeftY();
      rot = -driver.getRightX();
    }
    double speedScale = 5.0;

    double heading = rPose.getRotation().getRadians() + rot * 3.0 * dt * speedScale;

    double x = rPose.getX() + fwd * speedScale * dt;
    double y = rPose.getY() + strafe * speedScale * dt;

    Pose2d candidate = new Pose2d(x, y, Rotation2d.fromRadians(heading));
    rPose = enforceSelfClearance(rPose, candidate);

    if (VisionSimWorld.getSelfPose() == null) {
      VisionSimWorld.setSelfPose(rPose);
    }

    // Logger.recordOutput("VisionSim/RobotPose", rPose);
  }

  private Pose2d enforceSelfClearance(Pose2d prevPose, Pose2d candidate) {
    Pose2d selfPose = VisionSimWorld.getSelfPose();
    if (selfPose == null) {
      return candidate;
    }

    double centerX = selfPose.getX();
    double centerY = selfPose.getY();

    double prevDx = prevPose.getX() - centerX;
    double prevDy = prevPose.getY() - centerY;
    double candDx = candidate.getX() - centerX;
    double candDy = candidate.getY() - centerY;

    double halfX = Constants.ROBOT_X * 1.1;
    double halfY = Constants.ROBOT_Y * 1.1;

    boolean prevInside = Math.abs(prevDx) <= halfX && Math.abs(prevDy) <= halfY;
    boolean candInside = Math.abs(candDx) <= halfX && Math.abs(candDy) <= halfY;

    if (!candInside) {
      return candidate;
    }

    if (!prevInside) {
      return prevPose;
    }

    double prevDistSq = prevDx * prevDx + prevDy * prevDy;
    double candDistSq = candDx * candDx + candDy * candDy;

    if (candDistSq >= prevDistSq) {
      return candidate;
    } else {
      return prevPose;
    }
  }

  public static void setSelfPose(Pose2d p) {
    VisionSimWorld.setSelfPose(p);
  }
}
