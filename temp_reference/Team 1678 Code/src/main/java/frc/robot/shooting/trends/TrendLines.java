package frc.robot.shooting.trends;

import edu.wpi.first.math.Pair;
import frc.lib.math.LinearLineCalculator;
import java.util.List;

public class TrendLines {

	public static final LinearLineCalculator HUB_TREND_LINE = LinearLineCalculator.bestFit(List.of(
			// comp
			// Pair.of(1.967387, 1.046),
			// Pair.of(2.207408, 1.08),
			// Pair.of(2.21, 1.1),

			// // 29.7
			// // 30.800
			// Pair.of(2.57, 1.1),

			// // 30.933
			// // 32.1
			// Pair.of(2.663911, 1.107),

			// // 14.900
			// // 16.1
			// Pair.of(2.75, 1.2),

			// // 41.367
			// // 42.600
			// Pair.of(2.80631, 1.233),

			// // 27.33
			// // 28.567
			// Pair.of(3.0, 1.237),

			// 17.333
			// 18.600
			// Pair.of(3.5696633, 1.267) ,

			// Pair.of(4.18, 1.3),

			// 6.193
			// 7.659
			// Pair.of(4.5, 1.466)

			// epsilon

			// 36.845
			// 38.32
			Pair.of(6.4, 1.475),

			// 0.6
			// 2.135
			Pair.of(7.3, 1.535),

			// 16.608
			// 18.242
			Pair.of(8.06, 1.634),

			// 3.302
			// 5.202
			Pair.of(9.06, 1.9),

			// 10.505
			// 12.722
			Pair.of(10.2, 2.217) // ,

			// OZ
			// Pair.of(12.8, -1.0)

			));

	public static final LinearLineCalculator FERRY_TREND_LINE = LinearLineCalculator.bestFit(List.of(
			Pair.of(6.4, 1.475),

			// 0.6
			// 2.135
			Pair.of(7.3, 1.535),

			// 16.608
			// 18.242
			Pair.of(8.06, 1.634),

			// 3.302
			// 5.202
			Pair.of(9.06, 1.9),

			// 10.505
			// 12.722
			Pair.of(10.2, 2.217) // ,
			));
}
