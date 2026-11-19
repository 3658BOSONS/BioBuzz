package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.algorithm.Foresight
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer
import com.pedropathing.tuning.autotune.*
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner


object Tuning {
//	@JvmStatic
//	@Tuner
//	fun threeWheelTuner(): Procedure {
//		return ThreeWheelTuner()
//	}
	
	@JvmStatic
	@Tuner
	fun foresightTuner(): Procedure {
		return ForesightTuner({ hardwareMap ->
			ThreeWheelLocalizer(
				hardwareMap,
				Constants.localizerConfig
			)
		}, { hardwareMap -> Mecanum(hardwareMap, Constants.driveConfig) })
	}
	
	@Tuner
	fun tests(): Procedure {
		return Tests(
			{ hardwareMap -> Mecanum(hardwareMap, Constants.drivetrainConfig) },
			({ hardwareMap -> PinpointLocalizer(hardwareMap, Constants.localizerConfig) }),
			{ Foresight(Constants.foresightConfig) })
	}
}