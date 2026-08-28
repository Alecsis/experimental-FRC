// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public static final AngularVelocity kFreeSpeed = RPM.of(10000);
  private static final AngularVelocity tolerance = RPM.of(100);
  private static final double kJamStatorCurrentAmps = 70;
  private static final double kJamVelocityThresholdRadPerSec = RotationsPerSecond.of(5).in(RadiansPerSecond);

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter(
          Constants.currentMode == Constants.Mode.REAL ? new ShooterIOReal() : new ShooterIOSim());
    }
    return instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public double targetRPM = 0.0;
  private Agitate currentAgitate = Agitate.STOP;
  private indexing currentIndexing = indexing.STOP;

  /*
   * ---------------------------------------------------------------------------------------------
   * Shooter tuning mode.
   *
   * The whole mode is TWO ownership rules enforced right here, at the mechanism, rather than a
   * parallel set of commands fighting Superstructure's wanted-state arbitration:
   *
   *   FLYWHEEL -- while tuningMode is on, the operator's manual target owns it. Every AUTOMATIC
   *               writer (vision LUT, fixed fallback, STOWED's idle RPM, OFF's zero) goes through
   *               targetRPMShooter(), which becomes a no-op. No Superstructure state, present or
   *               future, can take the flywheel back by accident.
   *
   *   FEED PATH -- while a deliberate hold-to-feed action is running, it owns the indexer and
   *               agitator, and the automatic writers indexControl()/setAgitator() become no-ops
   *               for its duration.
   *
   * This is the same "one writer per mechanism, decided up front" rule Superstructure already
   * documents for the intake pivot and roller, applied to the two shooter mechanisms.
   *
   * Superstructure additionally refuses to START a shot while tuning is on (see its requestShot()),
   * so the automatic sequence never runs and nothing ever commands INDEX on its own.
   * ---------------------------------------------------------------------------------------------
   */

  /** Ceiling for a manually-dialled tuning target. Matches the production LUT clamp's upper bound. */
  private static final double kMaxTuningRPM = 6000.0;

  /** Default increment for the +/- tuning controls. */
  private static final double kDefaultTuningStepRPM = 50.0;

  private boolean tuningMode = false;
  private double tuningTargetRPM = 0.0;
  private double tuningStepRPM = kDefaultTuningStepRPM;
  private boolean tuningFeedActive = false;

  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;

  public enum Agitate {
    STOP(0),
    IN(0.35), // if shred lower 5%
    OUT(0.25);

    private final double percent;

    Agitate(double percent) {
      this.percent = percent;
    }

    public Voltage voltage() {
      return Volts.of(percent * 12);
    }
  }

  public enum indexing {
    INDEX(0.60),
    STOP(0),
    EJECT(-0.40);

    private final double percent;

    indexing(double percent) {
      this.percent = percent;
    }

    public Voltage voltage() {
      return Volts.of(percent * 12);
    }
  }

  /** Creates a new Shooter. Use {@link #getInstance()} instead of constructing directly. */
  private Shooter(ShooterIO io) {
    this.io = io;
  }

  /**
   * AUTOMATIC agitator writer -- what {@code Superstructure.periodic()} and the agitate commands
   * call. Ignored while a deliberate tuning feed owns the feed path; see the ownership note above.
   */
  public void setAgitator(Agitate state) {
    if (tuningFeedActive) {
      return;
    }
    applyAgitator(state);
  }

  /** Unconditional agitator write. Only the feed-path owner may call this. */
  private void applyAgitator(Agitate state) {
    io.setAgitatorVoltage(state.voltage().in(Volts));
    currentAgitate = state;
    Logger.recordOutput("Agitator State", state);
  }

  /** The agitator state most recently sent via {@link #setAgitator(Agitate)}. */
  public Agitate getAgitateState() {
    return currentAgitate;
  }

  public Command agitate() {
    return Commands.startEnd(
        () -> setAgitator(Agitate.IN),
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateReverse() {
    return Commands.startEnd(
        () -> setAgitator(Agitate.OUT),
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateStop() {
    return Commands.run(() -> setAgitator(Agitate.STOP));
  }

  /** Package-private (not private) so in-package JUnit tests can assert on it directly -- same
   *  reasoning as Intake's identical pattern. */
  boolean isJammed() {
    return inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps
        && inputs.indexVelocityRadsPerSec < kJamVelocityThresholdRadPerSec;
  }

  /** Test-only: forces isJammed()'s inputs, bypassing physics ShooterIOSim can't model (no
   *  load/obstruction). Package-private -- only for JUnit tests in this package. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real ShooterIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }

  public Command indexJam() {
    return Commands.sequence(
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)),
        Commands.waitUntil(this::isJammed),
        Commands.runOnce(() -> indexControl(Shooter.indexing.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)).repeatedly()
            .finallyDo(() -> indexControl(Shooter.indexing.EJECT)));
  }

  public void setRPMShooter(double rpm) {
    io.setShootVelocity(RPM.of(rpm).in(RadiansPerSecond));
  }

  /**
   * AUTOMATIC flywheel target -- vision LUT, fixed fallback, STOWED idle, OFF zero. Ignored while
   * tuning mode owns the flywheel, which is the single rule that stops the LUT or the fallback
   * overwriting a manually-dialled target without any caller needing to know tuning exists.
   */
  public void targetRPMShooter(double rpm) {
    if (tuningMode) {
      return;
    }
    targetRPM = rpm;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  /** Measured flywheel speed, RPM. */
  public double getActualRPM() {
    return RadiansPerSecond.of(inputs.shootVelocityRadsPerSec).in(RPM);
  }

  /* ------------------------------------------------------------------ tuning-mode surface ---- */

  /** Whether manual tuning currently owns the flywheel. */
  public boolean isTuningMode() {
    return tuningMode;
  }

  /**
   * Arms or disarms tuning mode.
   *
   * <p>Entering seeds the manual target from a STOP rather than from whatever the flywheel happened
   * to be doing, so arming the mode can never itself spin anything up. Exiting drops the ownership
   * rule, and the next {@code Superstructure.periodic()} tick -- which writes a flywheel target in
   * every state -- immediately reinstates normal policy, so no explicit hand-back is needed. Also
   * releases any feed the tuning action was holding, since nothing else would.
   */
  public void setTuningMode(boolean enabled) {
    if (enabled == tuningMode) {
      return;
    }
    tuningMode = enabled;
    releaseTuningFeed();
    if (enabled) {
      tuningTargetRPM = 0.0;
      targetRPM = 0.0;
    }
  }

  /**
   * Clears tuning mode outright. Called from the disabled transition so that arming tuning is
   * always a fresh, deliberate act: a mode left on before a disable must not silently resurrect on
   * the next enable, when the operator may be expecting normal match behaviour.
   */
  public void exitTuningMode() {
    setTuningMode(false);
  }

  /** The manually-dialled tuning target, RPM. Meaningful only while tuning mode is on. */
  public double getTuningTargetRPM() {
    return tuningTargetRPM;
  }

  /** Sets the manual tuning target. Clamped to [0, {@value #kMaxTuningRPM}]; inert outside tuning. */
  public void setTuningTargetRPM(double rpm) {
    tuningTargetRPM = MathUtil.clamp(rpm, 0.0, kMaxTuningRPM);
  }

  /** Increment/decrement size for the +/- tuning controls, RPM. */
  public double getTuningStepRPM() {
    return tuningStepRPM;
  }

  public void setTuningStepRPM(double stepRPM) {
    tuningStepRPM = MathUtil.clamp(stepRPM, 1.0, 1000.0);
  }

  /** Nudges the manual target by {@code multiplier} steps. Adjusts RPM only -- never feeds. */
  public void stepTuningTargetRPM(double multiplier) {
    setTuningTargetRPM(tuningTargetRPM + multiplier * tuningStepRPM);
  }

  /** Whether the deliberate hold-to-feed tuning action currently owns the feed path. */
  public boolean isTuningFeedActive() {
    return tuningFeedActive;
  }

  /**
   * HOLD-TO-FEED for shooter tuning: the only way a game piece is fed while tuning owns the shooter.
   *
   * <p>Deliberately a hold, and deliberately bound to a physical control rather than published as a
   * SmartDashboard command button -- a dashboard button is a latch by construction (it stays
   * pressed until something un-presses it), and a latched feed empties the hopper through an
   * untuned flywheel. Releasing runs {@code end()} on the next scheduler tick, which both drops
   * feed-path ownership and drives the indexer and agitator to STOP.
   *
   * <p>Inert unless tuning mode is armed, so the binding cannot feed during a match.
   */
  public Command tuningFeedCmd() {
    return Commands.startEnd(
        () -> {
          if (!tuningMode) {
            return;
          }
          tuningFeedActive = true;
          applyIndex(indexing.INDEX);
          applyAgitator(Agitate.IN);
        },
        this::releaseTuningFeed,
        this)
        .withName("ShooterTuningFeed");
  }

  private void releaseTuningFeed() {
    if (!tuningFeedActive) {
      return;
    }
    tuningFeedActive = false;
    applyIndex(indexing.STOP);
    applyAgitator(Agitate.STOP);
  }

  public void setOutputShooter(double percent) {
    io.setShootVoltage(percent * 12);
  }

  public void stopShooter() {
    targetRPMShooter(0);
  }

  public Command spin(double rpm) {
    return Commands.sequence(
        Commands.runOnce(() -> targetRPMShooter(rpm)),
        Commands.waitUntil(() -> shooterAtSpeed(rpm)));
  }

  public Command dashSpin() {
    return defer(() -> spin(targetRPM));
  }

  public boolean shooterAtSpeed(double targetRPM) {
    AngularVelocity current = RadiansPerSecond.of(inputs.shootVelocityRadsPerSec);
    return current.isNear(RPM.of(targetRPM), tolerance);
  }

  /**
   * AUTOMATIC indexer writer -- what {@code Superstructure.periodic()} and the index commands call.
   * Ignored while a deliberate tuning feed owns the feed path; see the ownership note above.
   */
  public void indexControl(indexing state) {
    if (tuningFeedActive) {
      return;
    }
    applyIndex(state);
  }

  /** Unconditional indexer write. Only the feed-path owner may call this. */
  private void applyIndex(indexing state) {
    io.setIndexVoltage(state.voltage().in(Volts));
    currentIndexing = state;
    Logger.recordOutput("Indexer State", state);
  }

  /** The indexer state most recently sent via {@link #indexControl(indexing)}. */
  public indexing getIndexerState() {
    return currentIndexing;
  }

  public Command index() {
    return Commands.startEnd(
        () -> indexControl(indexing.INDEX),
        () -> indexControl(indexing.STOP),
        this);
  }

  public Command indexEject() {
    return Commands.startEnd(
        () -> indexControl(indexing.EJECT),
        () -> indexControl(indexing.STOP),
        this);
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Current RPM",
        () -> RadiansPerSecond.of(inputs.shootVelocityRadsPerSec).in(RPM),
        null);

    builder.addDoubleProperty(
        "Current Volt",
        () -> inputs.shootAppliedVolts,
        null);

    builder.addDoubleProperty(
        "Target RPM",
        () -> targetRPM,
        null);

    // Writes the TUNING target, not the applied target: outside tuning mode this is inert (nothing
    // reads tuningTargetRPM), and inside it, it is the manual target that owns the flywheel. Typing
    // a number here can therefore never fight the state machine for the flywheel.
    builder.addDoubleProperty(
        "Dashboard RPM",
        () -> tuningTargetRPM,
        this::setTuningTargetRPM);

    builder.addDoubleProperty(
        "Error",
        () -> targetRPM - RadiansPerSecond.of(inputs.shootVelocityRadsPerSec).in(RPM),
        null);

    builder.addBooleanProperty("Shooter Tuning Mode",
        this::isTuningMode,
        this::setTuningMode);

    builder.addDoubleProperty("Tuning Step RPM", this::getTuningStepRPM, this::setTuningStepRPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (testJamOverrideActive) {
      inputs.indexStatorCurrentAmps = testJamStatorCurrentAmps;
      inputs.indexVelocityRadsPerSec = testJamVelocityRadsPerSec;
    }
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putBoolean(
        "Shooter/Index Stall", inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps);
    Logger.recordOutput("Shooter/Jammed", isJammed());

    // Tuning mode owns the applied flywheel target outright. targetRPMShooter() has already been
    // neutralised for every automatic writer, so this is simply where the manual value lands.
    if (tuningMode) {
      targetRPM = tuningTargetRPM;
    }

    if (targetRPM > 0) {
      setRPMShooter(targetRPM);
    } else {
      io.setShootVoltage(0);
    }

    Logger.recordOutput("Shooter Target RPM", targetRPM);
  }
}
