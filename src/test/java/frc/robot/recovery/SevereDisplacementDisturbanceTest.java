package frc.robot.recovery;

/** Severe displacement: ~2.0m lateral shove mid-path -- well past Constants.kMaxVisionJumpMeters. */
class SevereDisplacementDisturbanceTest extends PathDisturbanceSimTestBase {
    @Override
    protected double displacementMeters() {
        return 2.0;
    }
}
