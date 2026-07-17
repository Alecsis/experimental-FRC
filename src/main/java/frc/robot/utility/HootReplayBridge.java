package frc.robot.utility;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

/**
 * Owns the CTRE Phoenix 6 Hoot auto-log/auto-replay hookup for {@link HubActiveState}, isolated
 * into its own file so the general-purpose utility class doesn't carry a direct vendor import.
 */
class HootReplayBridge {
  private final HootAutoReplay autoReplay;

  HootReplayBridge(
      BooleanSupplier hubActiveGet, Consumer<Boolean> hubActiveSet,
      DoubleSupplier timeUntilSwapGet, DoubleConsumer timeUntilSwapSet) {
    this.autoReplay = new HootAutoReplay()
        .withBoolean("Hub/Active", hubActiveGet::getAsBoolean, val -> hubActiveSet.accept(val.value))
        .withDouble("Hub/Time Until Swap", timeUntilSwapGet::getAsDouble, val -> timeUntilSwapSet.accept(val.value));
  }

  boolean isReplay() {
    return Utils.isReplay();
  }

  void update() {
    autoReplay.update();
  }
}
