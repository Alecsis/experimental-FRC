package frc.robot.recovery;

/** Small defense bump: ~0.4m lateral shove mid-path. */
class SmallDisplacementDisturbanceTest extends PathDisturbanceSimTestBase {
    @Override
    protected double displacementMeters() {
        return 0.4;
    }
}
