package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.junit.jupiter.api.Test;

class PredictiveFieldStateLocalAccessAllianceTest {
  private static final double EPS = 1e-9;

  @Test
  void normalizeDynamicObjectsMirrorsRedSideCoordinatesWhenFlipEnabled() {
    List<DynamicObject> normalized =
        PredictiveFieldStateLocalAccess.normalizeDynamicObjects(
            List.of(fuelObject("fuel-red", Constants.FIELD_LENGTH - 1.0, 4.0, 0.6, -0.2)), true);

    assertEquals(1, normalized.size());
    DynamicObject object = normalized.get(0);
    assertEquals(1.0, object.pos.getX(), EPS);
    assertEquals(4.0, object.pos.getY(), EPS);
    assertEquals(-0.6, object.vel.getX(), EPS);
    assertEquals(-0.2, object.vel.getY(), EPS);
  }

  @Test
  void normalizeDynamicObjectsKeepsCoordinatesWhenFlipDisabled() {
    List<DynamicObject> normalized =
        PredictiveFieldStateLocalAccess.normalizeDynamicObjects(
            List.of(fuelObject("fuel-red", Constants.FIELD_LENGTH - 1.0, 4.0, 0.6, -0.2)), false);

    assertEquals(1, normalized.size());
    DynamicObject object = normalized.get(0);
    assertEquals(Constants.FIELD_LENGTH - 1.0, object.pos.getX(), EPS);
    assertEquals(0.6, object.vel.getX(), EPS);
  }

  @Test
  void normalizeDynamicObjectsDropsObjectsOutsideField() {
    List<DynamicObject> normalized =
        PredictiveFieldStateLocalAccess.normalizeDynamicObjects(
            List.of(fuelObject("off-field", -0.1, 2.0, 0.0, 0.0)), false);

    assertTrue(normalized.isEmpty());
  }

  @Test
  void allianceZonePredicateMatchesConfiguredBlueZone() {
    Translation2d inside = new Translation2d(1.0, Constants.FIELD_WIDTH * 0.5);
    Translation2d outside =
        new Translation2d(Constants.FIELD_LENGTH * 0.6, Constants.FIELD_WIDTH * 0.5);

    assertTrue(PredictiveFieldStateLocalAccess.inAllianceZoneBlue(inside));
    assertFalse(PredictiveFieldStateLocalAccess.inAllianceZoneBlue(outside));
    assertTrue(PredictiveFieldStateLocalAccess.inField(inside));
    assertFalse(PredictiveFieldStateLocalAccess.inField(new Translation2d(-0.1, 1.0)));
  }

  private static ShuttleRecoveryDynamicObjectDTO fuelObject(
      String id, double x, double y, double vx, double vy) {
    ShuttleRecoveryDynamicObjectDTO dto = new ShuttleRecoveryDynamicObjectDTO();
    dto.setId(id);
    dto.setType("fuel");
    dto.setX(x);
    dto.setY(y);
    dto.setVx(vx);
    dto.setVy(vy);
    dto.setAgeS(0.05);
    return dto;
  }
}
