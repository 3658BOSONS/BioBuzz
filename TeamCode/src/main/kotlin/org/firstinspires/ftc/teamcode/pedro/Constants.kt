package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.algorithm.Foresight
import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.config.Configuration
import com.pedropathing.controllers.Controller
import com.pedropathing.follower.Follower
import com.pedropathing.math.Matrix
import com.pedropathing.math.Vector2D
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.pedropathing.revhub.localizers.Encoder
import com.pedropathing.revhub.localizers.ThreeWheelConfig
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


object Constants {
	var localizerConfig: ThreeWheelConfig = ThreeWheelConfig(Configuration { c: ThreeWheelConfig? ->
		c!!.leftEncoderName.set("frontLeftMotor")
		c.rightEncoderName.set("backRightMotor")
		c.strafeEncoderName.set("frontRightMotor")
		c.leftPodY.set(6.439672650322374)
		c.rightPodY.set(-4.809404598477303)
		c.strafePodX.set(-6.69296768957344)
		c.forwardTicksToInches.set(7.153125886953374E-4)
		c.strafeTicksToInches.set(7.39565197294424E-4)
		c.turnTicksToRadians.set(6.358855689889268E-5)
		c.leftEncoderDirection.set(Encoder.REVERSE)
		c.rightEncoderDirection.set(Encoder.REVERSE)
		c.strafeEncoderDirection.set(Encoder.FORWARD)
	})
	
	var driveConfig = MecanumConfig(
		Configuration { c: MecanumConfig? ->
			c!!.frontLeftName.set("frontLeftMotor")
			c.backLeftName.set("backLeftMotor")
			c.frontRightName.set("frontRightMotor")
			c.backRightName.set("backRightMotor")
			c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE)
			c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE)
			c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD)
			c.backRightDirection.set(DcMotorSimple.Direction.FORWARD)
		}
	)
	
	var foresightConfig: ForesightConfig = ForesightConfig(
		Configuration { c: ForesightConfig? ->
			val primaryTranslationalForward = Controller.proportional(0.7373320197332774)
			val secondaryTranslationalForward = Controller.proportional(0.2724244794170242)
			val primaryTranslationalLateral = Controller.proportional(2.081594286977066)
			val secondaryTranslationalLateral = Controller.proportional(0.7690934678142872)
			
			c!!.forwardTranslational.set(
				Controller.piecewise(secondaryTranslationalForward)
					.put(2.5, primaryTranslationalForward)
			)
			c.strafeTranslational.set(
				Controller.piecewise(secondaryTranslationalLateral)
					.put(2.5, primaryTranslationalLateral)
			)
			
			c.coast.set(Controller.proportionalFeedforward(0.01976682344540139))
			c.brake.set(Controller.proportionalFeedforward(0.01680179992859118))
			
			c.headingFeedback.set(Controller.proportional(259.4832033586185))
			c.headingBrakeCoefficients.set(
				Vector2D.cartesian(
					0.26670791051386833,
					-0.044211625536312
				)
			)
			
			c.linearBrakeCoefficients.set(Matrix.diag(0.12372403837275424, 0.0228849393346195))
			c.quadraticBrakeCoefficients.set(
				Matrix.diag(
					0.003597492349658888,
					-2.000722002636723E-4
				)
			)
			
			c.maxAchievableForwardVelocity.set(23.938199804424368)
			c.maxAchievableStrafeVelocity.set(48.925921930677575)
			c.naturalForwardDeceleration.set(29.37502362171604)
			c.naturalStrafeDeceleration.set(48.58305955217726)
		}
	)

	fun create(h: HardwareMap?): Follower {
		return Follower(
			ThreeWheelLocalizer(h, this.localizerConfig),
			Mecanum(h, this.driveConfig),
			Foresight(this.foresightConfig)
		);
	}
}