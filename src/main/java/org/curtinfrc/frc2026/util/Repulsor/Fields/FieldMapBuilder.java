package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElementModel;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameObject;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameObjectType;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.PrimitiveObject;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public final class FieldMapBuilder {
  public enum CategorySpec {
    kScore,
    kCollect,
    kEndgame
  }

  public static final class ElementSpec {
    Alliance alliance = Alliance.kBlue;
    int capacity = 1;
    Pose3d pose = new Pose3d();
    final List<PrimitiveObject> primitives = new ArrayList<>();
    Predicate<GameObject> filter = go -> true;
    RepulsorSetpoint related;
    CategorySpec category;
  }

  private final FieldTracker ft;
  private final List<GameElement> elements = new ArrayList<>();
  private ElementSpec spec;

  public FieldMapBuilder(FieldTracker ft) {
    this.ft = Objects.requireNonNull(ft);
  }

  public FieldMapBuilder begin() {
    spec = new ElementSpec();
    return this;
  }

  public FieldMapBuilder alliance(Alliance a) {
    spec.alliance = a;
    return this;
  }

  public FieldMapBuilder capacity(int c) {
    spec.capacity = c;
    return this;
  }

  public FieldMapBuilder pose(Pose3d p) {
    spec.pose = p;
    return this;
  }

  public FieldMapBuilder rotate(double rollRad, double pitchRad, double yawRad) {
    Pose3d p = spec.pose;
    spec.pose = new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad));
    return this;
  }

  public FieldMapBuilder translate(double dx, double dy, double dz) {
    Pose3d p = spec.pose;
    spec.pose = new Pose3d(p.getX() + dx, p.getY() + dy, p.getZ() + dz, p.getRotation());
    return this;
  }

  public FieldMapBuilder primitivePipe(
      Distance radius, double rollRad, double pitchRad, double yawRad) {
    Pose3d p = spec.pose;
    spec.primitives.add(
        ft
        .new Pipe(
            new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad)),
            radius,
            Radians.of(yawRad)));
    return this;
  }

  public FieldMapBuilder filter(Predicate<GameObject> f) {
    spec.filter = f != null ? f : (go -> true);
    return this;
  }

  public FieldMapBuilder filterCoral() {
    spec.filter = go -> go.getType() == GameObjectType.kCoral;
    return this;
  }

  public FieldMapBuilder filterAlgae() {
    spec.filter = go -> go.getType() == GameObjectType.kAlgae;
    return this;
  }

  public FieldMapBuilder related(RepulsorSetpoint sp) {
    spec.related = sp;
    return this;
  }

  public FieldMapBuilder add() {
    PrimitiveObject[] prim = spec.primitives.toArray(new PrimitiveObject[0]);
    GameElementModel model = ft.new GameElementModel(spec.pose, prim);
    elements.add(
        ft
        .new GameElement(
            spec.alliance, spec.capacity, model, spec.filter, spec.related, spec.category));
    spec = null;
    return this;
  }

  public FieldMapBuilder bulk(
      List<Pose3d> poses,
      Alliance alliance,
      int capacity,
      Distance radius,
      double yawRad,
      java.util.function.Predicate<GameObject> filter,
      List<RepulsorSetpoint> related,
      CategorySpec category) {
    int n = Math.min(poses.size(), related.size());
    for (int i = 0; i < n; i++) {
      begin()
          .alliance(alliance)
          .capacity(capacity)
          .pose(poses.get(i))
          .primitivePipe(radius, 0, 0, yawRad)
          .filter(filter)
          .related(related.get(i))
          .category(category)
          .add();
    }
    return this;
  }

  public FieldMapBuilder mirrorX(double xAxis) {
    List<GameElement> mirrored = new ArrayList<>();
    for (GameElement e : elements) {
      Pose3d p = e.getModel().getPosition();
      Pose3d mp = new Pose3d(2 * xAxis - p.getX(), p.getY(), p.getZ(), p.getRotation());
      PrimitiveObject[] prims = e.getModel().getComposition();
      List<PrimitiveObject> nprims = new ArrayList<>();
      for (PrimitiveObject pr : prims) {
        if (pr instanceof FieldTracker.Pipe) {
          FieldTracker.Pipe pipe = (FieldTracker.Pipe) pr;
          Pose3d pp = pipe.getPosition();
          nprims.add(
              ft
              .new Pipe(
                  new Pose3d(2 * xAxis - pp.getX(), pp.getY(), pp.getZ(), pp.getRotation()),
                  pipe.getRadius(),
                  pipe.getAngle()));
        }
      }
      GameElementModel model = ft.new GameElementModel(mp, nprims.toArray(new PrimitiveObject[0]));
      GameElement m =
          ft
          .new GameElement(
              e.getAlliance(),
              e.getMaxContained(),
              model,
              e::filter,
              e.getRelatedPoint().orElse(null),
              e.getCategory());
      mirrored.add(m);
    }
    elements.addAll(mirrored);
    return this;
  }

  public FieldMapBuilder category(CategorySpec c) {
    spec.category = c;
    return this;
  }

  public GameElement[] build() {
    return elements.toArray(new GameElement[0]);
  }

  public static Distance small() {
    return Meters.of(0.20);
  }

  public static Distance medium() {
    return Meters.of(0.30);
  }

  public static Distance large() {
    return Meters.of(0.40);
  }

  public static Distance tiny() {
    return Meters.of(0.10);
  }
}
