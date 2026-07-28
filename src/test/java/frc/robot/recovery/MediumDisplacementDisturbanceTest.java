package frc.robot.recovery;

/** Medium defense hit: ~1.0m lateral shove mid-path -- right at Constants.kMaxVisionJumpMeters. */
class MediumDisplacementDisturbanceTest extends PathDisturbanceSimTestBase {
    @Override
    protected double displacementMeters() {
        return 1.0;
    }
}
