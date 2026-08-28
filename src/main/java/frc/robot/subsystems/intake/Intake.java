// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final double kJamStatorCurrentAmps = 80;
  private static final double kJamVelocityThresholdRadPerSec = RotationsPerSecond.of(5).in(RadiansPerSecond);
  private static final double kJamRecoveryPulseSeconds = 0.3;
  private static final double kJamRecoveryCooldownSeconds = 0.3;

  /*
   * -----------------------------------------------------------------------------------------
   * Homing tuning constants.
   *
   * TUNE FROM A REAL HOMING LOG, NOT FROM SIMULATION. The three detector constants below are
   * bring-up defaults chosen from the physics of the mechanism plus the simulated trace; only
   * kHomingVoltage and kHardstopStatorCurrentAmps are carried over unchanged from the
   * previously-proven implementation. Re-derive the rest from Intake/PivotStatorCurrentAmps and
   * Intake/PivotVelocity in a real homing wpilog (see the telemetry block in periodic()).
   * -----------------------------------------------------------------------------------------
   */

  /**
   * Seek voltage. UNCHANGED and deliberately not re-derived: +3.0 V against the stock
   * {@code CounterClockwise_Positive} pivot inversion is the polarity that has already homed this
   * exact mechanism on real hardware. Nothing in this file may reverse it.
   */
  private static final double kHomingVoltage = 3.0;

  /**
   * Stator current that, sustained, means the pivot is pushing on something. UNCHANGED at 65 A on
   * purpose: signal rate, detector shape and the position guard all change in this pass, so the
   * threshold is held fixed to keep the real-hardware comparison to one variable.
   */
  private static final double kHardstopStatorCurrentAmps = 65;

  /**
   * "The mechanism is not moving." The pivot free-runs at roughly 500 deg/s at {@link
   * #kHomingVoltage} (Kraken X60 free speed 6000 RPM x 3/12 V / {@link Constants#kIntakePivotReduction}
   * = 86 RPM = 514 deg/s; the simulated seek measured 8.7 rad/s = 499 deg/s), so 30 deg/s is about
   * 6% of free speed -- unambiguously stalled, and far above any plausible encoder noise floor.
   */
  private static final double kHardstopMaxPivotVelocityDegPerSec = 30.0;

  /**
   * How long the full hardstop signature (current AND near-stall) must hold CONTINUOUSLY before a
   * home is declared. Three robot loops. This is what makes a single-sample acceleration spike
   * unable to declare home, and it is only meaningful because {@link IntakeIOReal} raises the
   * current signal above the loop rate -- at the stock 4 Hz CAN 2.0 rate three consecutive loop
   * samples could be three reads of the SAME stale frame.
   */
  private static final double kHardstopQualifiedSeconds = 0.06;

  /**
   * Dead time at the start of a seek during which no hardstop can be declared, however the current
   * looks. The pivot is stationary at seek start, so the breakaway/acceleration inrush satisfies
   * BOTH "high current" and "near-stall" at once -- persistence alone cannot separate it from a
   * real stop. This window must outlast breakaway and stay well inside
   * {@link #kHomingSeekTimeoutSeconds}. 0.15 s is a bring-up default: MEASURE the real breakaway
   * time from Intake/PivotVelocity's first crossing of {@link #kHardstopMaxPivotVelocityDegPerSec}.
   */
  private static final double kHomingStartupGraceSeconds = 0.15;

  /** Seek stage time bound. Unchanged from the previously-proven implementation. */
  private static final double kHomingSeekTimeoutSeconds = 1.0;

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake(
          Constants.currentMode == Constants.Mode.REAL ? new IntakeIOReal() : new IntakeIOSim());
    }
    return instance;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean homed = false;
  private boolean deploy = false;
  private boolean sysIdActive = false;
  private Roller currentRoller = Roller.STOP;
  private boolean jamRecoveryActive = false;
  private double jamRecoveryStartTimestamp = 0.0;
  private double jamRecoveryCooldownUntilTimestamp = 0.0;

  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;

  private boolean pivotCurrentOverrideActive = false;
  private double pivotCurrentOverrideAmps;
  private boolean pivotVelocityOverrideActive = false;
  private double pivotVelocityOverrideRadsPerSec;

  /*
   * Homing observability. homingState describes the most recent homing ATTEMPT; homed describes
   * whether a usable zero reference exists at all. They are deliberately separate: a failed retry
   * after an earlier good home leaves homingState = FAILED_TIMEOUT with homed still true, because
   * discarding a known-good reference on a failed retry would be strictly worse than keeping it.
   */
  private HomingState homingState = HomingState.NOT_HOMED;
  private double homingSeekStartTimestamp = 0.0;
  /** Timestamp the current unbroken run of hardstop signature began, or NaN if not qualifying. */
  private double homingQualifiedSinceTimestamp = Double.NaN;
  /** Latched by the seek stage's own execute() so the zeroing stage reads the SAME tick's verdict. */
  private boolean homingSeekQualified = false;
  private boolean homingAboveCurrent = false;
  private boolean homingLowVelocity = false;
  private double homingQualifiedDurationSeconds = 0.0;
  /** Rising-edge-logged count of position requests refused because no home reference exists. */
  private long blockedPivotPositionRequests = 0;
  /** How many times the encoder reference has been zeroed. One per SUCCESSFUL home, never more. */
  private long pivotZeroCount = 0;

  /**
   * Homing lifecycle, as logged to {@code Intake/HomingState}.
   *
   * <ul>
   *   <li>{@code NOT_HOMED} -- no seek has run yet this power cycle.
   *   <li>{@code SEEKING} -- a seek is driving {@link #kHomingVoltage} and watching for the stop.
   *   <li>{@code HOMED} -- the last seek qualified a hardstop and zeroed the reference.
   *   <li>{@code FAILED_TIMEOUT} -- the last seek ran its full timeout without qualifying.
   *   <li>{@code INTERRUPTED} -- the last seek was cancelled before reaching either outcome.
   * </ul>
   */
  public enum HomingState {
    NOT_HOMED, SEEKING, HOMED, FAILED_TIMEOUT, INTERRUPTED
  }

  public enum Roller {
    STOP(0),
    INTAKE(800),
    EJECT(-500);

    private final double rpm;

    Roller(double rpm) {
      this.rpm = rpm;
    }

    public double getRPM() {
      return rpm;
    }
  }

  public enum PivotState {
    HOME(-10),
    STOW(-245),
    AGITATE(-95),
    DOWN(-10);

    private final double deg;

    PivotState(double deg) {
      this.deg = deg;
    }

    public Angle angle() {
      return Degrees.of(deg);
    }
  }

  private final SysIdRoutine m_rollerSysId;

  /** Creates a new Intake. Use {@link #getInstance()} instead of constructing directly. */
  private Intake(IntakeIO io) {
    this.io = io;
    m_rollerSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> Logger.recordOutput("Intake/RollerSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> io.setRollerVoltage(volts.in(Volts)),
            log -> {
              log.motor("Roller")
                  .voltage(Volts.of(inputs.rollerAppliedVolts))
                  .angularVelocity(RadiansPerSecond.of(inputs.rollerVelocityRadsPerSec))
                  .angularPosition(Radians.of(inputs.rollerPositionRads));
            },
            this));
    SmartDashboard.putBoolean("Intake/Zeroed", false);
  }

  public void setRoller(Roller state) {
    if (state != Roller.INTAKE) {
      // STOP or EJECT always wins immediately, aborting any in-progress recovery pulse/cooldown.
      jamRecoveryActive = false;
      jamRecoveryCooldownUntilTimestamp = 0.0;
      applyRoller(state);
      return;
    }

    // Resuming INTAKE right after an EJECT pulse sweeps the roller back through low/negative
    // velocity under elevated current -- the same electrical signature isJammed() looks for.
    // Suppress re-arming the detector until that resume transient has had time to clear, or every
    // recovery pulse immediately re-triggers itself.
    boolean inCooldown = Timer.getFPGATimestamp() < jamRecoveryCooldownUntilTimestamp;

    if (!jamRecoveryActive && !inCooldown && isJammed()) {
      jamRecoveryActive = true;
      jamRecoveryStartTimestamp = Timer.getFPGATimestamp();
    }

    if (jamRecoveryActive) {
      if (Timer.getFPGATimestamp() - jamRecoveryStartTimestamp < kJamRecoveryPulseSeconds) {
        applyRoller(Roller.EJECT); // recovery pulse in progress
        return;
      }
      jamRecoveryActive = false; // pulse elapsed, fall through to resume INTAKE
      jamRecoveryCooldownUntilTimestamp = Timer.getFPGATimestamp() + kJamRecoveryCooldownSeconds;
    }

    applyRoller(Roller.INTAKE);
  }

  private void applyRoller(Roller state) {
    if (state == Roller.STOP) {
      io.setRollerVoltage(0);
    } else {
      io.setRollerVelocity(RotationsPerSecond.of(state.getRPM() / 60.0).in(RadiansPerSecond));
    }
    currentRoller = state;
    Logger.recordOutput("Intake/TargetState", state);
    Logger.recordOutput("Intake/JamRecoveryActive", jamRecoveryActive);
  }

  /** The roller state most recently sent via {@link #setRoller(Roller)}. */
  public Roller getRollerState() {
    return currentRoller;
  }

  /** Whether the intake currently holds a game piece. Sim: IronMaple's simulated field/collision
   *  ground truth (IntakeIOSim). Real: not yet wired to a sensor -- IntakeIOReal never sets this,
   *  so it always reads false on real hardware today. */
  public boolean hasGamePiece() {
    return inputs.hasGamePiece;
  }

  /** Test-only: injects one simulated game piece directly into IntakeIOSim's IntakeSimulation,
   *  bypassing field/collision physics. No-op (returns false) if not running against IntakeIOSim,
   *  or if IntakeIOSim's IntakeSimulation hasn't been lazily constructed yet. Package-private --
   *  only for JUnit tests in this package. */
  boolean provideGamePieceForTest() {
    return io instanceof IntakeIOSim sim && sim.addGamePieceForTest();
  }

  private boolean isJammed() {
    return inputs.rollerStatorCurrentAmps > kJamStatorCurrentAmps
        && inputs.rollerVelocityRadsPerSec < kJamVelocityThresholdRadPerSec;
  }

  /** Test-only: forces isJammed()'s inputs, bypassing physics IntakeIOSim can't model (no
   *  load/obstruction). Package-private -- only for JUnit tests in this package. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real IntakeIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }

  /**
   * Whether a roller SysId routine is currently characterizing the roller. This is the one and
   * only case where a Command -- not Superstructure -- owns the roller, so Superstructure's direct
   * {@code periodic()} roller writes must stand down or they would overwrite the characterization
   * sweep. Every other Intake Command ({@link #homing()}, {@link #agitatePivot()}) drives the pivot
   * only, which is why roller ownership is deliberately NOT inferred from "some Command requires
   * Intake": doing that let a stale roller request survive for the whole homing window.
   */
  public boolean isRollerSysIdActive() {
    return sysIdActive;
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> sysIdActive = true)
        .andThen(m_rollerSysId.quasistatic(direction))
        .finallyDo(() -> {
          sysIdActive = false;
          setRoller(Roller.STOP);
        });
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> sysIdActive = true)
        .andThen(m_rollerSysId.dynamic(direction))
        .finallyDo(() -> {
          sysIdActive = false;
          setRoller(Roller.STOP);
        });
  }

  public void goTo(PivotState pos) {
    if (!commandPivotPosition(pos.angle().in(Radians))) {
      return;
    }
    Logger.recordOutput("Intake/TargetPivotAngle", pos.angle());
  }

  public void goToDegrees(double deg) {
    commandPivotPosition(Degrees.of(deg).in(Radians));
  }

  /**
   * THE authoritative failed-home safety guard, and the single choke point every closed-loop pivot
   * request in this class passes through ({@link #goTo}, {@link #goToDegrees}, and therefore
   * {@link #agitatePivot()} and every {@code Superstructure.setIntakePivot()} call). It is placed
   * here rather than at each call site precisely so a future caller cannot bypass it by accident.
   *
   * <p><b>The rule.</b> While no home reference exists, the pivot may be driven by the homing
   * voltage and it may be left neutral, but it may NOT be sent to a MotionMagic position. The
   * RotorSensor boots with a relative zero that does not represent the physical pivot angle, so
   * before homing, "position 0" is wherever the pivot happened to be at power-up -- and the robot
   * powers up STOWED/UP. Honouring {@code PivotState.DOWN} against that boot zero would drive the
   * mechanism to an arbitrary physical angle at full closed-loop authority (kP 70). Refusing the
   * request instead leaves whatever the caller last commanded electrically in force, which after a
   * timed-out seek is the explicit 0 V the seek exit stage applied: unhomed, and neutral.
   *
   * <p>Deliberately does NOT command 0 V itself. Doing so would make this a second writer racing
   * whichever command legitimately owns the pivot, which is the exact failure class
   * {@code docs/claudex/architecture.md} records; the guard job is to withhold a request, not to
   * issue one.
   *
   * @return true if the request was actually issued to the IO layer
   */
  private boolean commandPivotPosition(double positionRads) {
    if (!homed) {
      if (blockedPivotPositionRequests == 0) {
        DriverStation.reportWarning(
            "Intake pivot position request refused: the pivot has never homed, so its encoder zero"
                + " does not represent a physical angle. Run intake.homing() successfully first.",
            false);
      }
      blockedPivotPositionRequests++;
      return false;
    }
    io.setPivotPosition(positionRads);
    return true;
  }

  /** Whether a hardstop-detected home reference currently exists. */
  public boolean isHomed() {
    return homed;
  }

  /** The lifecycle state of the most recent homing attempt. See {@link HomingState}. */
  public HomingState getHomingState() {
    return homingState;
  }

  public Command agitatePivot() {
    return Commands.sequence(
        Commands.runOnce(() -> goTo(PivotState.AGITATE), this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> goTo(PivotState.DOWN), this),
        Commands.waitSeconds(0.5))
        .repeatedly()
        .finallyDo(() -> goTo(PivotState.DOWN));
  }

  public Command eject() {
    return Commands.startEnd(
        () -> setRoller(Roller.EJECT),
        () -> setRoller(Roller.STOP),
        this);
  }

  public Command startRoller() {
    return Commands.startEnd(
        () -> setRoller(Roller.INTAKE),
        () -> setRoller(Roller.STOP),
        this);
  }

  /**
   * RAW current-only hardstop signal -- one sample, no filter. Kept public and unchanged because it
   * is the instantaneous electrical reading, useful on its own in telemetry and tests.
   * <b>It is not the homing decision.</b> A seek qualifies a hardstop through
   * {@link #evaluateHardstopQualification()}, which additionally requires near-stall motion, short
   * continuous persistence, and that the startup grace window has elapsed.
   */
  public boolean hardstop() {
    return inputs.pivotStatorCurrentAmps > kHardstopStatorCurrentAmps;
  }

  /** Whether the pivot is moving slowly enough to be considered stalled against something. */
  private boolean pivotNearStall() {
    return Math.abs(Radians.of(inputs.pivotVelocityRadsPerSec).in(Degrees))
        < kHardstopMaxPivotVelocityDegPerSec;
  }

  /**
   * The physically meaningful hardstop detector, evaluated once per seek tick.
   *
   * <p>"Pushing hard" and "not moving" must be true TOGETHER and STAY true. Current alone cannot
   * distinguish a stop from the inrush of accelerating the arm; near-stall alone cannot distinguish
   * a stop from an arm that has not broken away yet. Requiring both, for
   * {@link #kHardstopQualifiedSeconds}, after {@link #kHomingStartupGraceSeconds}, is what makes a
   * single acceleration spike unable to end the seek.
   *
   * <p>Each robot loop contributes one sample, and {@link IntakeIOReal} publishes the pivot current
   * and velocity above the loop rate specifically so that those samples are FRESH rather than
   * repeated reads of one stale CAN frame -- persistence over stale data would prove nothing.
   */
  private boolean evaluateHardstopQualification() {
    double now = Timer.getFPGATimestamp();
    homingAboveCurrent = hardstop();
    homingLowVelocity = pivotNearStall();

    boolean withinGrace = now - homingSeekStartTimestamp < kHomingStartupGraceSeconds;
    if (withinGrace || !homingAboveCurrent || !homingLowVelocity) {
      homingQualifiedSinceTimestamp = Double.NaN;
      homingQualifiedDurationSeconds = 0.0;
      return false;
    }

    if (Double.isNaN(homingQualifiedSinceTimestamp)) {
      homingQualifiedSinceTimestamp = now;
    }
    homingQualifiedDurationSeconds = now - homingQualifiedSinceTimestamp;
    return homingQualifiedDurationSeconds >= kHardstopQualifiedSeconds;
  }

  /** Seek stage initialize(): arms the detector and starts the grace window. */
  private void beginHomingSeek() {
    homingState = HomingState.SEEKING;
    homingSeekStartTimestamp = Timer.getFPGATimestamp();
    homingQualifiedSinceTimestamp = Double.NaN;
    homingQualifiedDurationSeconds = 0.0;
    homingSeekQualified = false;
  }

  /**
   * Seek stage execute(): drives the seek voltage, then evaluates and LATCHES the detector.
   * Latching (rather than re-reading the detector in the next stage) is what guarantees the zeroing
   * stage acts on the same tick sensor snapshot that ended the seek.
   */
  private void seekTick() {
    io.setPivotVoltage(kHomingVoltage);
    homingSeekQualified = evaluateHardstopQualification();
  }

  public Command homing() {
    return Commands.sequence(
        // A FunctionalCommand rather than run().until(): initialize() must arm the detector on the
        // same tick the seek voltage first goes out, and a leading runOnce stage would insert an
        // extra scheduler tick before any voltage was applied.
        new FunctionalCommand(
            this::beginHomingSeek,
            this::seekTick,
            interrupted -> {
            },
            () -> homingSeekQualified,
            this)
            .withTimeout(kHomingSeekTimeoutSeconds),
        Commands.runOnce(() -> {
          io.setPivotVoltage(0.0);
          // Only a QUALIFIED hardstop is a home. Reaching this stage by timeout instead means the
          // seek never found the stop, and re-zeroing here would install a zero reference at an
          // arbitrary angle -- every later setPivotPosition() would then target the wrong physical
          // position, which is a worse failure than simply not being homed. Leaving the previous
          // reference alone keeps a good earlier home if there was one, and leaving homed false
          // lets a retry (teleop's RobotModeTriggers.teleop().onTrue(homing())) run again -- and
          // now also leaves commandPivotPosition()'s guard refusing position requests, so a failed
          // home can no longer be followed by a DOWN command against the boot zero.
          if (homingSeekQualified) {
            io.setPivotEncoderPosition(0.0);
            pivotZeroCount++;
            homed = true;
            deploy = false;
            homingState = HomingState.HOMED;
          } else {
            homingState = HomingState.FAILED_TIMEOUT;
          }
          SmartDashboard.putBoolean("Intake/Zeroed", homed);
        }, this),
        // Guarded by commandPivotPosition() -- after a failed seek this is a no-op and the pivot
        // stays at the 0 V the stage above applied.
        Commands.runOnce(() -> goTo(PivotState.DOWN), this))
        .finallyDo(interrupted -> {
          // Whole-sequence finalizer. Interrupting during the seek stage above skips the zeroing
          // stage entirely, leaving the last setPivotVoltage(kHomingVoltage) request LATCHED:
          // CTRE control requests persist until superseded, and IntakeIOSim models that faithfully
          // (setPivotVoltage leaves pivotClosedLoop false, so updateInputs keeps re-applying the
          // stored volts every tick). "Home Intake" is the first command in all 15 autos, so any
          // cancel inside its first second -- teleopInit(), a disable, a test cancel -- lands
          // exactly in that window, and Superstructure cannot rescue it: it starts in OFF, and OFF
          // is the one state whose periodic() never calls setIntakePivot().
          //
          // Deliberately conditional on interruption: on normal completion the stage above has
          // already handed the pivot to setPivotPosition(DOWN), and an unconditional zero here
          // would supersede that MotionMagic request and drop the pivot limp.
          if (interrupted) {
            io.setPivotVoltage(0.0);
            homingState = HomingState.INTERRUPTED;
            homingQualifiedSinceTimestamp = Double.NaN;
            homingQualifiedDurationSeconds = 0.0;
            homingSeekQualified = false;
          }
        });
  }

  /**
   * Test-only: the pivot voltage the IO layer most recently reported applying, as of the last
   * {@link #periodic()}. Package-private and read-only -- same seam style as
   * {@link #forceJamConditionForTest}, and the only way a test can prove {@link #homing()} leaves
   * no latched voltage request behind without reaching into the private IO.
   */
  double pivotAppliedVoltsForTest() {
    return inputs.pivotAppliedVolts;
  }

  /** Test-only: whether a hardstop-detected home has been recorded. Package-private, read-only. */
  boolean homedForTest() {
    return homed;
  }

  /**
   * Test-only: forces {@link #hardstop()}'s input. IntakeIOSim's arm physics always reaches the
   * hardstop current within a few ticks and cannot be steered away from it on demand, so the
   * timeout branch of {@link #homing()} is otherwise unreachable in simulation. Same seam style as
   * {@link #forceJamConditionForTest} -- overrides a sensor reading only, and is inert unless a
   * test switches it on.
   */
  void forcePivotStatorCurrentForTest(double statorCurrentAmps) {
    pivotCurrentOverrideActive = true;
    pivotCurrentOverrideAmps = statorCurrentAmps;
  }

  /** Test-only: stops overriding the pivot current, resuming real IntakeIOSim physics. */
  void clearPivotCurrentOverrideForTest() {
    pivotCurrentOverrideActive = false;
  }

  /**
   * Test-only: forces {@link #pivotNearStall()}'s input. Together with
   * {@link #forcePivotStatorCurrentForTest(double)} this is what lets a test present the two halves
   * of the hardstop signature INDEPENDENTLY -- a high-current sample while the arm is still moving
   * fast is the "startup acceleration spike" case, which IntakeIOSim's near-massless arm cannot
   * produce on its own (measured: 6 A while free-running, 91.5 A only once pinned at the stop).
   */
  void forcePivotVelocityForTest(double velocityRadsPerSec) {
    pivotVelocityOverrideActive = true;
    pivotVelocityOverrideRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding the pivot velocity, resuming real IntakeIOSim physics. */
  void clearPivotVelocityOverrideForTest() {
    pivotVelocityOverrideActive = false;
  }

  /** Test-only: how many closed-loop pivot requests {@link #commandPivotPosition} has refused. */
  long blockedPivotPositionRequestsForTest() {
    return blockedPivotPositionRequests;
  }

  /**
   * Test-only: how many times {@code setPivotEncoderPosition(0)} has been issued. IntakeIOSim
   * implements that call as a no-op (its physics already tracks true position), so counting here is
   * the only way a simulation test can prove a successful home zeroes the reference exactly ONCE.
   */
  long pivotZeroCountForTest() {
    return pivotZeroCount;
  }

  /**
   * Test-only: discards the home reference, returning the subsystem to its power-up posture.
   * {@link Intake} is a singleton with no reset hook, so a test that needs to observe the
   * never-homed guard after some earlier test in the same JVM already homed has no other way to get
   * back there. Deliberately does not touch the IO layer or any output.
   */
  void clearHomeReferenceForTest() {
    homed = false;
    homingState = HomingState.NOT_HOMED;
    homingSeekQualified = false;
    homingQualifiedSinceTimestamp = Double.NaN;
    homingQualifiedDurationSeconds = 0.0;
    blockedPivotPositionRequests = 0;
    SmartDashboard.putBoolean("Intake/Zeroed", false);
  }

  public Command stopRoller() {
    return Commands.runOnce(() -> setRoller(Roller.STOP));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (testJamOverrideActive) {
      inputs.rollerStatorCurrentAmps = testJamStatorCurrentAmps;
      inputs.rollerVelocityRadsPerSec = testJamVelocityRadsPerSec;
    }
    if (pivotCurrentOverrideActive) {
      inputs.pivotStatorCurrentAmps = pivotCurrentOverrideAmps;
    }
    if (pivotVelocityOverrideActive) {
      inputs.pivotVelocityRadsPerSec = pivotVelocityOverrideRadsPerSec;
    }
    Logger.processInputs("Intake", inputs);

    SmartDashboard.putBoolean("Intake/Roller Stall", inputs.rollerStatorCurrentAmps > kJamStatorCurrentAmps);
    SmartDashboard.putNumber("Intake/Pivot Deg", Radians.of(inputs.pivotPositionRads).in(Degrees));
    logHomingTelemetry();
  }

  /**
   * Publishes everything needed to read a real homing attempt back out of a wpilog and re-derive
   * the detector constants, without having to correlate several unrelated topics by hand.
   *
   * <p>{@code HomingState} is the lifecycle of the last ATTEMPT; {@code Zeroed} is whether a usable
   * reference exists at all. The three {@code Homing*} booleans/durations are the detector's own
   * inputs and internal state, so a log shows exactly WHY a seek did or did not qualify -- which of
   * the two halves of the signature was missing, and for how long the pair actually held.
   *
   * <p>Angular units are degrees and degrees per second throughout, matching
   * {@link #kHardstopMaxPivotVelocityDegPerSec} so the log can be compared against the constant
   * directly.
   */
  private void logHomingTelemetry() {
    Logger.recordOutput("Intake/HomingState", homingState);
    Logger.recordOutput("Intake/Zeroed", homed);
    Logger.recordOutput("Intake/PivotDeg", Radians.of(inputs.pivotPositionRads).in(Degrees));
    Logger.recordOutput(
        "Intake/PivotVelocity", Radians.of(inputs.pivotVelocityRadsPerSec).in(Degrees));
    Logger.recordOutput("Intake/PivotAppliedVolts", inputs.pivotAppliedVolts);
    Logger.recordOutput("Intake/PivotStatorCurrentAmps", inputs.pivotStatorCurrentAmps);
    Logger.recordOutput("Intake/PivotSupplyCurrentAmps", inputs.pivotSupplyCurrentAmps);
    Logger.recordOutput("Intake/HomingAboveCurrent", homingAboveCurrent);
    Logger.recordOutput("Intake/HomingLowVelocity", homingLowVelocity);
    Logger.recordOutput("Intake/HomingQualifiedDuration", homingQualifiedDurationSeconds);
    Logger.recordOutput("Intake/BlockedPivotPositionRequests", blockedPivotPositionRequests);
    Logger.recordOutput("Intake/PivotZeroCount", pivotZeroCount);
    SmartDashboard.putString("Intake/HomingState", homingState.toString());
    SmartDashboard.putBoolean("Intake/Zeroed", homed);
  }
}
