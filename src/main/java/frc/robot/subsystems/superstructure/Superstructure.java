// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

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
    OFF, INTAKING, EJECTING, STOWED, ALIGNING, SHOOTING
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
    mAlignTimeoutSeconds = Double.POSITIVE_INFINITY;
    mSettleDelaySeconds = 0.1;
    mFeedTimeoutSeconds = Double.POSITIVE_INFINITY;
    mShotInProgress = true;
    mWantedState = SuperstructureState.ALIGNING;
  }

  /** Requests the shooting sequence with a bounded spin-up/settle/feed window (autonomous). */
  public void requestShoot(double timeoutSeconds) {
    mAlignTimeoutSeconds = 1.0;
    mSettleDelaySeconds = 0.3;
    mFeedTimeoutSeconds = timeoutSeconds;
    mShotInProgress = true;
    mWantedState = SuperstructureState.ALIGNING;
  }

  /** Requests active floor intake. */
  public void requestIntake() {
    mWantedState = SuperstructureState.INTAKING;
  }

  /** Requests reverse/eject: intake down, roller and indexer running backwards (unjam / dump). */
  public void requestEject() {
    mWantedState = SuperstructureState.EJECTING;
  }

  /** Returns to the idle/ready posture -- shoot button released, intaking finished, shot completed, etc. */
  public void requestStow() {
    mWantedState = SuperstructureState.STOWED;
    mShotInProgress = false;
  }

  public SuperstructureState getSystemState() {
    return mSystemState;
  }

  private void setState(SuperstructureState state) {
    if (mSystemState != state) {
      mStateStartTimestamp = Timer.getFPGATimestamp();
    }
    mSystemState = state;
    if (state != SuperstructureState.ALIGNING && state != SuperstructureState.SHOOTING && mAgitateCommand != null) {
      mAgitateCommand.cancel();
      mAgitateCommand = null;
    }
  }

  /** Schedules Intake's own pivot-agitate routine if it isn't already running. Owned/mechanism-controlled by Intake. */
  private void ensureAgitating() {
    if (mAgitateCommand == null || !mAgitateCommand.isScheduled()) {
      mAgitateCommand = intake.agitatePivot();
      CommandScheduler.getInstance().schedule(mAgitateCommand);
    }
  }

  private void updateShooterRPM() {
    if (shooter.shooterTuningModeEnable) {
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

  /** Bridges the state machine into a bounded, self-finishing Command for autonomous/NamedCommands. */
  public Command shootingSequence(double timeoutSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> requestShoot(timeoutSeconds)),
        Commands.waitUntil(() -> !mShotInProgress));
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
        intake.setRoller(Roller.STOP);
        break;

      case STOWED:
        shooter.indexControl(Shooter.indexing.STOP);
        shooter.setAgitator(Shooter.Agitate.STOP);
        shooter.targetRPMShooter(kIdleRPM);
        intake.setRoller(Roller.STOP);
        intake.goTo(Intake.PivotState.DOWN);
        break;

      case INTAKING:
        intake.goTo(Intake.PivotState.DOWN);
        intake.setRoller(Roller.INTAKE);
        // Matches the old teleop intake binding: agitator runs outward while intaking so
        // incoming fuel doesn't pack against the indexer.
        shooter.setAgitator(Shooter.Agitate.OUT);
        break;

      case EJECTING:
        intake.goTo(Intake.PivotState.DOWN);
        intake.setRoller(Roller.EJECT);
        shooter.indexControl(Shooter.indexing.EJECT);
        shooter.setAgitator(Shooter.Agitate.OUT);
        break;

      case ALIGNING:
        if (mWantedState != SuperstructureState.ALIGNING) {
          setState(mWantedState);
          break;
        }
        ensureAgitating();
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
        ensureAgitating();
        updateShooterRPM();
        if (timeInState >= mSettleDelaySeconds) {
          shooter.indexControl(Shooter.indexing.INDEX);
          shooter.setAgitator(Shooter.Agitate.IN);
        }
        if (timeInState >= mFeedTimeoutSeconds) {
          requestStow();
          setState(SuperstructureState.STOWED);
        }
        break;
    }
  }
}
