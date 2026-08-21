package frc.robot.recovery;

/** Control run: same path, same harness, zero injected displacement -- isolates the disturbance's own effect from this codebase's already-documented baseline tracking-error behavior. */
class NoDisturbanceControlTest extends PathDisturbanceSimTestBase {
    @Override
    protected double displacementMeters() {
        return 0.0;
    }
}
