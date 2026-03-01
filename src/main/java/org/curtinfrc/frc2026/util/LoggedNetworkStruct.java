package org.curtinfrc.frc2026.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedNetworkStruct<T extends StructSerializable> extends LoggedNetworkInput
    implements Supplier<T> {
  private final String key;
  private final StructEntry<T> entry;
  private T defaultValue;
  private T value;

  @SuppressWarnings("unchecked")
  /**
   * Creates a new LoggedNetworkStruct, for handling a struct input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedNetworkStruct(String key, T defaultValue) {
    this.defaultValue = defaultValue;
    this.value = defaultValue;

    this.key = key;
    try {
      // It doesn't like this but this should be an invariant guaranteed by StructSerializable
      var struct = (Struct<T>) defaultValue.getClass().getField("struct").get(defaultValue);
      this.entry =
          NetworkTableInstance.getDefault().getStructTopic(key, struct).getEntry(defaultValue);
    } catch (Exception e) {
      throw new IllegalArgumentException("Provided struct type or default value is invalid.", e);
    }
    Logger.registerDashboardInput(this);
  }

  /**
   * Updates the default value, which is used if no value in NT is found.
   *
   * @param defaultValue The new default value.
   */
  public void setDefault(T defaultValue) {
    this.defaultValue = defaultValue;
    entry.set(entry.get(defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   *
   * @param value The new value.
   */
  public void set(T value) {
    entry.set(value);
  }

  /**
   * Returns the current value.
   *
   * @return The current value.
   */
  public T get() {
    return value;
  }

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(removeSlash(key), value);
        }

        public void fromLog(LogTable table) {
          value = table.get(removeSlash(key), defaultValue);
        }
      };

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.get(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
