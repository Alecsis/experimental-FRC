# Lesson 14: Full Autonomous Tuning Workflow

## Learning objective
Walk the real September Tuning Playbook end to end on the academy stack as a dry run, so that
when real hardware access returns, every step of the actual playbook (`docs/
Path_Following_Tuning_Readiness_Audit.md`) is a rerun of something you've already done once, not
a first attempt under time pressure.

## Concepts introduced
No new concepts — this lesson is integration and sequencing. It composes every prior lesson's
skill into one ordered pass:
1. Pre-flight sanity (Lesson 1's boot-level check, scaled to the full academy chassis)
2. SysId characterization (Lesson 4) on the academy motor/module stack
3. Apply fitted feedforward (Lesson 3's concepts, real fitted numbers this time)
4. Retune drive Slot0 off that feedforward (Lesson 2's intuition, applied with real numbers)
5. Chassis-level controller check (Lesson 11's saturation awareness) with the academy chassis's
   real traction ceiling (an academy-invented value, not copied from `TunerConstants`) as the
   target rather than a guess
6. `discretize()` A/B (Lesson 7's concept, run as an actual before/after comparison this time)
7. Full path-following regression pass (Lesson 12's harness, run before and after every change
   above)

## Existing code to reuse
All of it — this lesson doesn't introduce new production-code references, it's the point where
you should be able to *predict* which production file a given step maps to before checking:
step 2 → `CommandSwerveDrivetrain.java:109-161`'s `SysIdRoutine`s; step 4 →
`TunerConstants.java`'s `driveGains`; step 5 → `CommandSwerveDrivetrain.java:364-368`'s
`PIDConstants`; step 6 → the confirmed-absent `ChassisSpeeds.discretize()` call; step 7 →
`AutoRegressionTestBase`. If you can't name the mapping without looking it up, that's a signal to
revisit the corresponding earlier lesson before finishing this one.

## New simulation command / demo
`academy.auto.FullTuningWorkflowTest` — not a single test, but an ordered sequence of the prior
lessons' harnesses run back-to-back with each step's output feeding the next (fitted SysId numbers
become the feedforward config, the retuned Slot0 feeds the chassis-level check, etc.), producing
one before/after comparison at the end (pre-flight academy defaults vs. fully-tuned academy stack).

## What to observe in Phoenix Tuner X
N/A (simulation) — but this lesson's step ordering *is* the September playbook's ordering, so
treat each step's Tuner X note from its source lesson (Lessons 1-11) as still applicable when this
workflow is eventually rerun against real hardware.

## What to observe in AdvantageScope
A single before/after comparison layout: pre-flight (untuned academy defaults) vs. final (fully
tuned) tracking error over the same path from Lesson 12, plus the SysId fit quality plot from
Lesson 4 and the discretize A/B from Lesson 7, all in one saved layout — this becomes the
template for the real September before/after comparison.

## Common failure modes
- Reordering steps (e.g. tuning chassis PID before applying feedforward) — the whole point of
  this lesson is that order matters, and doing it out of order here is cheap practice for not
  doing it out of order in September when it's expensive.
- Treating this as "redo Lesson 4/Lesson 2/etc. individually" rather than as one continuous
  pass where each step's output is the next step's input — the integration *is* the lesson.
- Skipping the discretize A/B because it "seems minor" compared to the SysId/gains work — it's
  included precisely because the production audit found it silently absent; don't let this
  dry run repeat that omission.

## Small experiments to build intuition
1. Run the full sequence once in the documented order, then run it again with steps 3 and 4
   swapped (retune Slot0 before applying feedforward) — compare the final tracking-error result
   to see concretely why the playbook's ordering isn't arbitrary.
2. Deliberately skip step 6 (discretize) entirely and compare the final regression pass's
   tracking error against the version that includes it.
3. Introduce an injected sensor fault (e.g. Lesson 10's vision outlier) partway through the final
   regression pass and confirm the workflow's earlier tuning work isn't what breaks — a fusion
   problem should show up as a fusion problem, not get misdiagnosed as a bad Slot0 retune.

## Success criteria
`FullTuningWorkflowTest`'s before/after comparison shows a clear, explainable improvement, you can
narrate all 7 steps and their file-path mappings from memory, and you can articulate at least one
thing this dry run makes you want to watch for specifically once real hardware access returns.
This is the last lesson — completing it means the academy's goal (September playbook familiarity)
has been met.
