// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * Singleton Superstructure. Owns a centralized, 254-style state machine that arbitrates Shooter +
 * Intake + Vision directly from {@link #periodic()} instead of composing long-running Commands.
 * Callers request a state via {@code requestX()}; {@link #periodic()} decides each tick's actual
 * {@link SuperstructureState} and commands subsystems accordingly. {@link #shootCmd()} and
 * {@link #shootingSequence(double)} are thin Command bridges kept for the existing RobotContainer
 * bindings and PathPlanner's NamedCommands, which both require real Command objects.
 */
public class Superstructure extends SubsystemBase {
  private static final double kIdleRPM = 700;

  public enum SuperstructureState {
    OFF, INTAKING, BOUNCING, EJECTING, STOWED, ALIGNING, SHOOTING
  }

  /** Operator-board intents in priority order. The maintained controls are physical source of truth. */
  public enum OperatorIntent {
    STOWED, INTAKING, BOUNCING, FALLBACK_SHOT, VISION_SHOT, EJECTING
  }

  private enum ShotMode {
    VISION, FIXED_FALLBACK
  }

  private static Superstructure instance;

  /** Creates (on first call) or returns the Superstructure Singleton. */
  public static Superstructure getInstance(CommandSwerveDrivetrain drivetrain) {
    if (instance == null) {
      instance = new Superstructure(drivetrain);
    }
    return instance;
  }

  private final Shooter shooter = Shooter.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Vision vision;

  private SuperstructureState mSystemState = SuperstructureState.OFF;
  private SuperstructureState mWantedState = SuperstructureState.OFF;
  private double mStateStartTimestamp = 0.0;
  private boolean mShotInProgress = false;
  private ShotMode mShotMode = ShotMode.VISION;
  /** The mode the in-flight shot actually spun up for; compared against mShotMode to catch a
   *  vision<->fallback switch made while already feeding. */
  private ShotMode mActiveShotMode = ShotMode.VISION;
  /**
   * True only while {@code mWantedState} holds the value the operator-policy default command last
   * wrote. Every {@code requestX()} entry point clears it, so nothing but the policy itself can
   * set it -- which is what lets {@link #clearOperatorRequest()} neutralize operator intent
   * without ever being able to touch an autonomous, NamedCommand, or dashboard-bridge request.
   */
  private boolean mOperatorRequestActive = false;

  /* Align/settle/feed timing -- teleop holds until released (infinite), auto is bounded. */
  private double mAlignTimeoutSeconds = Double.POSITIVE_INFINITY;
  private double mSettleDelaySeconds = 0.1;
  private double mFeedTimeoutSeconds = Double.POSITIVE_INFINITY;

  private Command mAgitateCommand;

  /** Creates a new Superstructure. Use {@link #getInstance(CommandSwerveDrivetrain)} instead of constructing directly. */
  private Superstructure(CommandSwerveDrivetrain drivetrain) {
    this.vision = Vision.getInstance(drivetrain);
  }

  /** Requests the shooting sequence, feeding continuously until {@link #requestStow()} (teleop hold-to-shoot). */
  public void requestShoot() {
    requestShot(ShotMode.VISION, Double.POSITIVE_INFINITY, 0.1, Double.POSITIVE_INFINITY);
  }

  /** Requests the shooting sequence with a bounded spin-up/settle/feed window (autonomous). */
  public void requestShoot(double timeoutSeconds) {
    requestShot(ShotMode.VISION, 1.0, 0.3, timeoutSeconds);
  }

  /** Requests the operator fallback shot: 1600 Kraken rotor RPM, then the normal feed path. */
  public void requestFallbackShot() {
    requestShot(ShotMode.FIXED_FALLBACK, Double.POSITIVE_INFINITY, 0.1, Double.POSITIVE_INFINITY);
  }

  private void requestShot(ShotMode mode, double alignTimeoutSeconds, double settleDelaySeconds,
      double feedTimeoutSeconds) {
    mOperatorRequestActive = false;
    mAlignTimeoutSeconds = alignTimeoutSeconds;
    mSettleDelaySeconds = settleDelaySeconds;
    mFeedTimeoutSeconds = feedTimeoutSeconds;
    mShotMode = mode;
    mShotInProgress = true;
    mWantedState = SuperstructureState.ALIGNING;
  }

  /*
   * The bare requestX() methods above and below are STATE REQUESTS, not actions: they write
   * mWantedState and return, leaving periodic() to decide what actually runs. They carry no
   * Command requirement of their own, so a request made outside a Superstructure-requiring Command
   * can be superseded on the very next tick by whatever currently owns the state -- in teleop,
   * that is the operator policy default command re-asserting the live control board. Call them
   * from inside a Command that requires Superstructure (as every bridge below does) when the
   * request needs to hold.
   */

  /** Requests active floor intake. */
  public void requestIntake() {
    mOperatorRequestActive = false;
    mWantedState = SuperstructureState.INTAKING;
  }

  /** Requests the legacy-equivalent pivot bounce while the intake roller stays active. */
  public void requestBounce() {
    mOperatorRequestActive = false;
    mWantedState = SuperstructureState.BOUNCING;
  }

  /** Requests reverse/eject: intake down, roller and indexer running backwards (unjam / dump). */
  public void requestEject() {
    mOperatorRequestActive = false;
    mWantedState = SuperstructureState.EJECTING;
  }

  /** Returns to the idle/ready posture -- shoot button released, intaking finished, shot completed, etc. */
  public void requestStow() {
    mOperatorRequestActive = false;
    mWantedState = SuperstructureState.STOWED;
    mShotInProgress = false;
  }

  public SuperstructureState getSystemState() {
    return mSystemState;
  }

  /** Resolves live operator controls without mutable latch state. Higher-priority momentaries win. */
  public static OperatorIntent resolveOperatorIntent(boolean fallbackShot, boolean visionShot,
      boolean eject, boolean bounce, boolean intake) {
    if (eject) {
      return OperatorIntent.EJECTING;
    }
    if (visionShot) {
      return OperatorIntent.VISION_SHOT;
    }
    if (fallbackShot) {
      return OperatorIntent.FALLBACK_SHOT;
    }
    if (bounce) {
      return OperatorIntent.BOUNCING;
    }
    if (intake) {
      return OperatorIntent.INTAKING;
    }
    return OperatorIntent.STOWED;
  }

  /**
   * The operator policy: a never-ending, level-triggered command that reads the live physical
   * control board every tick and applies the resulting intent. Installed as Superstructure's
   * DEFAULT command (see {@code OperatorControls}), which is what gives the maintained switches
   * their lifecycle guarantees -- none of which an edge-triggered {@code Trigger.whileTrue} can
   * provide:
   *
   * <ul>
   *   <li><b>Already ON at enable.</b> Nothing here waits for a false-&gt;true edge. The scheduler
   *       schedules a default command on the first enabled tick and this reads the switches as
   *       they physically are, so a toggle left ON while disabled takes effect on its own.
   *   <li><b>Disable then re-enable.</b> Same mechanism: the scheduler drops the default command
   *       while disabled and re-schedules it on re-enable, where it re-reads the live switches.
   *   <li><b>Interruption.</b> When any other Superstructure Command (a NamedCommand, a
   *       SmartDashboard button) finishes and releases the requirement, the scheduler re-schedules
   *       the default command automatically. The still-physically-ON toggle reasserts with no
   *       rescheduling logic here and no remembered latch -- the old {@code ctrlBtn} problem.
   * </ul>
   *
   * <p><b>Mechanism behavior is teleop-only, structurally.</b> An earlier attempt at a default
   * command drove the state machine to STOWED throughout autonomous (700 RPM flywheel, pivot
   * commanded DOWN from the instant of enable, fighting the "Home Intake" NamedCommand). The
   * {@code isTeleopEnabled()} gate below is the fix: outside teleop this command issues no request
   * at all, so {@code mWantedState} is left exactly as autonomous set it. Board and sim-mirror
   * controls alike are therefore inert during autonomous -- not by convention, but because the
   * only code path that could act on them is closed.
   *
   * <p>STOWED is simply the zero-active-control teleop intent, so no separate teleop-start stow
   * command is needed and correctness does not depend on trigger registration order.
   *
   * <p><b>The intent is stored, but {@code execute()} is gated -- so ending must clean up.</b>
   * Each tick writes a persistent {@code mWantedState} that {@link #periodic()} keeps applying in
   * EVERY robot mode, while the body above goes inert the moment teleop ends. Without the
   * {@code finallyDo} below, that asymmetry lets an operator request outlive the only code that
   * could revise it: a disable taken mid-EJECT re-issues EJECT on the first re-enabled tick even
   * though the button was released while disabled, and an interrupting Superstructure command that
   * does not immediately write its own request inherits the previous operator state for its whole
   * duration. Ending here therefore relinquishes the stored request through the normal WPILib
   * command end path, which covers disabled cancellation, interruption by another
   * Superstructure-requiring command, explicit cancellation, and any other end -- no scheduler
   * polling and no second competing command. Teleop EXIT is handled separately and does not depend
   * on this one firing; see {@link #clearOperatorRequest()}.
   */
  public Command operatorPolicyCmd(BooleanSupplier fallbackShot, BooleanSupplier visionShot,
      BooleanSupplier eject, BooleanSupplier bounce, BooleanSupplier intake) {
    return Commands.run(() -> {
      if (!DriverStation.isTeleopEnabled()) {
        return;
      }
      switch (resolveOperatorIntent(
          fallbackShot.getAsBoolean(), visionShot.getAsBoolean(), eject.getAsBoolean(),
          bounce.getAsBoolean(), intake.getAsBoolean())) {
        case EJECTING -> requestEject();
        case VISION_SHOT -> requestShoot();
        case FALLBACK_SHOT -> requestFallbackShot();
        case BOUNCING -> requestBounce();
        case INTAKING -> requestIntake();
        case STOWED -> requestStow();
      }
      // Claim the wanted state just written. Deliberately AFTER the requestX() call, which cleared
      // this same flag -- that ordering is what makes the flag mean "the policy wrote the value
      // currently in mWantedState" rather than "the policy ran at some point".
      mOperatorRequestActive = true;
    }, this)
        .finallyDo(interrupted -> clearOperatorRequest())
        .withName("OperatorPolicy");
  }

  /**
   * Relinquishes operator-derived intent, and ONLY operator-derived intent.
   *
   * <p>Called from two independent places, because neither alone covers the whole problem:
   * the operator policy's own end/interruption path (above), and {@code Robot.teleopExit()} via
   * {@code RobotContainer} -- the latter so the invariant "no operator-derived wanted state
   * survives teleop" holds on a direct teleop-&gt;autonomous transition, where nothing guarantees
   * the default command is interrupted before autonomous starts running.
   *
   * <p>The {@code mOperatorRequestActive} guard is the reason this cannot repeat the regression
   * described in {@link #operatorPolicyCmd}: a bare unconditional {@code requestStow()} here would
   * fire every time an autonomous NamedCommand interrupted the (inert) default command, driving
   * the machine to STOWED -- 700 RPM flywheel, pivot commanded DOWN -- in the middle of autonomous.
   * Because every {@code requestX()} entry point clears the flag, any non-operator writer owns
   * {@code mWantedState} from the instant it writes it and this method becomes a no-op. Nothing
   * here writes STOWED continuously; it is a single relinquish at a lifecycle boundary.
   */
  public void clearOperatorRequest() {
    if (mOperatorRequestActive) {
      requestStow();
    }
  }

  private void setState(SuperstructureState state) {
    boolean leavingShot = isShootingState(mSystemState) && !isShootingState(state);
    if (mSystemState != state) {
      mStateStartTimestamp = Timer.getFPGATimestamp();
    }
    mSystemState = state;
    if (leavingShot) {
      // A shot leaves the flywheel at its vision/LUT (or fixed fallback) target. With maintained
      // toggles, releasing the momentary shoot button lands in INTAKING/BOUNCING -- neither of
      // which owns the shooter RPM -- so without this the flywheel would stay spun up at the shot
      // target indefinitely. Done here rather than in those two states so autonomous is untouched:
      // every auto leaves a shot into STOWED, which already commands kIdleRPM itself.
      shooter.targetRPMShooter(kIdleRPM);
    }
    if (state != SuperstructureState.ALIGNING && state != SuperstructureState.SHOOTING
        && state != SuperstructureState.BOUNCING && mAgitateCommand != null) {
      mAgitateCommand.cancel();
      mAgitateCommand = null;
    }
  }

  private static boolean isShootingState(SuperstructureState state) {
    return state == SuperstructureState.ALIGNING || state == SuperstructureState.SHOOTING;
  }

  /*
   * ---------------------------------------------------------------------------------------------
   * Intake ownership policy.
   *
   * Superstructure drives Intake straight from periodic(), so the CommandScheduler's requirement
   * arbitration cannot protect an Intake Command from it. Ownership is therefore declared
   * explicitly, PER MECHANISM, rather than inferred from "some Command requires Intake":
   *
   *   PIVOT  -- owned by whichever Command currently requires Intake (intake.homing() seeking the
   *             hardstop, the bounce routine, pivot-side tooling). Superstructure yields entirely.
   *             mAgitateCommand is the one exception: Superstructure scheduled it itself as its
   *             own BOUNCING actuator, so it is not "somebody else".
   *
   *   ROLLER -- owned by Superstructure at all times, except while a roller SysId sweep is
   *             characterizing (Intake.isRollerSysIdActive()). homing() and agitatePivot() drive
   *             the pivot ONLY and never touch the roller, so suppressing roller writes for them
   *             does not avoid a conflict -- it just freezes whatever the roller was last told,
   *             letting a stale INTAKE/EJECT request run through the entire homing window.
   *
   * Each mechanism therefore has exactly one writer at any instant, decided up front instead of by
   * last-write-wins.
   * ---------------------------------------------------------------------------------------------
   */

  /** Schedules Intake's own pivot-agitate routine if it isn't already running. Owned/mechanism-controlled by Intake. */
  private void ensureAgitating() {
    if (mAgitateCommand != null && mAgitateCommand.isScheduled()) {
      return;
    }
    // agitatePivot() requires Intake, so scheduling it would interrupt whatever else owns the
    // pivot -- most importantly intake.homing(), which must reach the hardstop before any pivot
    // position request means anything. Yield and retry next tick instead of yanking the mechanism
    // away mid-home.
    if (pivotOwnedByOtherCommand()) {
      return;
    }
    mAgitateCommand = intake.agitatePivot();
    CommandScheduler.getInstance().schedule(mAgitateCommand);
  }

  /** Whether a Command other than our own bounce routine currently owns the intake pivot. */
  private boolean pivotOwnedByOtherCommand() {
    Command owner = CommandScheduler.getInstance().requiring(intake);
    return owner != null && owner != mAgitateCommand;
  }

  private void setIntakePivot(Intake.PivotState pos) {
    if (!pivotOwnedByOtherCommand()) {
      intake.goTo(pos);
    }
  }

  private void setIntakeRoller(Roller state) {
    // Deliberately NOT gated on pivot ownership -- see the ownership policy note above. Only a
    // roller SysId sweep may take the roller away from the state machine.
    if (!intake.isRollerSysIdActive()) {
      intake.setRoller(state);
    }
  }

  /**
   * Drives the shooter feed path to a hard stop. Called on every entry to (and every pre-settle
   * tick of) the shot phases so no previously-commanded feed direction -- most importantly
   * EJECTING's {@code indexing.EJECT} -- can survive into a spin-up.
   */
  private void stopFeed() {
    shooter.indexControl(Shooter.indexing.STOP);
    shooter.setAgitator(Shooter.Agitate.STOP);
  }

  private void updateShooterRPM() {
    if (mShotMode == ShotMode.FIXED_FALLBACK) {
      shooter.targetRPMShooter(1600);
    } else if (shooter.shooterTuningModeEnable) {
      shooter.targetRPMShooter(shooter.getTargetRPM());
    } else {
      vision.calculateRPM().ifPresent(rpm -> {
        shooter.targetRPMShooter(rpm);
        Logger.recordOutput("Shooter/TargetRPM", rpm);
      });
    }
  }

  /** Bridges the state machine into a Command for the teleop hold-to-shoot binding ({@code whileTrue}). */
  public Command shootCmd() {
    return Commands.startEnd(this::requestShoot, this::requestStow, this);
  }

  /** Hold-to-intake: floor intake while held, stow on release. */
  public Command intakeCmd() {
    return Commands.startEnd(this::requestIntake, this::requestStow, this);
  }

  /** Hold-to-eject: intake down with roller and indexer reversed while held, stow on release. */
  public Command ejectCmd() {
    return Commands.startEnd(this::requestEject, this::requestStow, this);
  }

  /** Self-finishing: return everything to the idle/ready posture. */
  public Command stowCmd() {
    return Commands.runOnce(this::requestStow, this);
  }

  /**
   * Bridges the state machine into a bounded, self-finishing Command for autonomous/NamedCommands.
   * The {@code runOnce} declares {@code this} as a requirement (unlike a bare no-arg
   * {@code Commands.runOnce}) so the composed sequence requires Superstructure like every other
   * bridge Command here -- without it, a concurrently-scheduled {@link #shootCmd()}/
   * {@link #ejectCmd()} (e.g. a SmartDashboard button pressed mid-autonomous) would not be
   * cancelled and would run alongside this, both writing {@code mWantedState} on alternating
   * scheduler ticks.
   */
  public Command shootingSequence(double timeoutSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> requestShoot(timeoutSeconds), this),
        Commands.waitUntil(() -> !mShotInProgress));
  }

  /**
   * Bridges the state machine into a bounded, self-finishing Command for autonomous/NamedCommands.
   * {@link #intakeCmd()} is a hold-forever startEnd built for a teleop button binding -- inside a
   * PathPlanner "parallel" block (which compiles to a plain ParallelCommandGroup, requiring every
   * branch to finish) it never releases the group, silently stalling the auto after its first path
   * segment. This wraps it with a timeout so it always finishes on its own.
   */
  public Command intakeSequence(double timeoutSeconds) {
    return intakeCmd().withTimeout(timeoutSeconds);
  }

  @Override
  public void periodic() {
    if (mSystemState != SuperstructureState.ALIGNING && mSystemState != SuperstructureState.SHOOTING) {
      setState(mWantedState);
    }

    double timeInState = Timer.getFPGATimestamp() - mStateStartTimestamp;

    switch (mSystemState) {
      case OFF:
        shooter.targetRPMShooter(0);
        shooter.indexControl(Shooter.indexing.STOP);
        shooter.setAgitator(Shooter.Agitate.STOP);
        setIntakeRoller(Roller.STOP);
        break;

      case STOWED:
        shooter.indexControl(Shooter.indexing.STOP);
        shooter.setAgitator(Shooter.Agitate.STOP);
        shooter.targetRPMShooter(kIdleRPM);
        setIntakeRoller(Roller.STOP);
        setIntakePivot(Intake.PivotState.DOWN);
        break;

      case INTAKING:
        setIntakePivot(Intake.PivotState.DOWN);
        setIntakeRoller(Roller.INTAKE);
        // Resuming INTAKING after a momentary eject must not leave the indexer running backwards
        // -- EJECTING is the only state that sets indexing.EJECT and nothing else cleared it.
        // No-op for autonomous: no auto enters INTAKING from EJECTING or SHOOTING.
        shooter.indexControl(Shooter.indexing.STOP);
        // Matches the old teleop intake binding: agitator runs outward while intaking so
        // incoming fuel doesn't pack against the indexer.
        shooter.setAgitator(Shooter.Agitate.OUT);
        break;

      case BOUNCING:
        // Legacy toggle 5: bounce the pivot while the roller keeps running. agitatePivot() owns
        // the pivot (it requires Intake); the roller stays here so jam recovery is serviced every
        // tick rather than once per bounce cycle.
        ensureAgitating();
        setIntakeRoller(Roller.INTAKE);
        shooter.indexControl(Shooter.indexing.STOP);
        shooter.setAgitator(Shooter.Agitate.OUT);
        break;

      case EJECTING:
        setIntakePivot(Intake.PivotState.DOWN);
        setIntakeRoller(Roller.EJECT);
        shooter.indexControl(Shooter.indexing.EJECT);
        shooter.setAgitator(Shooter.Agitate.OUT);
        break;

      case ALIGNING:
        if (mWantedState != SuperstructureState.ALIGNING) {
          setState(mWantedState);
          break;
        }
        // Pre-shot means NOT feeding, unconditionally. Entering from EJECTING would otherwise
        // carry indexing.EJECT and Agitate.OUT straight through spin-up, so establish safe feed
        // outputs before the shooter is commanded anywhere. No-op for autonomous, which only ever
        // reaches ALIGNING from STOWED (already STOP/STOP).
        stopFeed();
        // Commit the shot mode this alignment is acquiring speed for. SHOOTING compares against
        // this to detect an operator switching between the vision and fallback targets mid-shot.
        mActiveShotMode = mShotMode;
        ensureAgitating();
        setIntakeRoller(Roller.INTAKE);
        updateShooterRPM();
        if (shooter.shooterAtSpeed(shooter.getTargetRPM()) || timeInState >= mAlignTimeoutSeconds) {
          setState(SuperstructureState.SHOOTING);
        }
        break;

      case SHOOTING:
        if (mWantedState != SuperstructureState.ALIGNING) {
          setState(mWantedState);
          break;
        }
        if (mShotMode != mActiveShotMode) {
          // The operator moved between the vision/LUT target and the fixed fallback target while
          // already feeding. The flywheel is about to be commanded somewhere else, so the feed
          // must stop in this same tick -- never feed through a target change -- and the shot must
          // go back through the pre-shot phase to acquire and settle at the NEW target before any
          // further fuel is indexed.
          stopFeed();
          setState(SuperstructureState.ALIGNING);
          break;
        }
        ensureAgitating();
        setIntakeRoller(Roller.INTAKE);
        updateShooterRPM();
        if (timeInState >= mSettleDelaySeconds) {
          shooter.indexControl(Shooter.indexing.INDEX);
          shooter.setAgitator(Shooter.Agitate.IN);
        } else {
          // Explicit rather than relying on ALIGNING's stopFeed() still being in effect.
          stopFeed();
        }
        if (timeInState >= mFeedTimeoutSeconds) {
          requestStow();
          setState(SuperstructureState.STOWED);
        }
        break;
    }
  }
}
