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

import edu.wpi.first.wpilibj2.command.Command;
import java.util.EnumSet;

public abstract class Behaviour {
  public abstract String name();

  public abstract int priority();

  public abstract boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx);

  public abstract Command build(BehaviourContext ctx);
}
