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
	var localizerConfig: ThreeWheelConfig = ThreeWheelConfig(Configuration { c: ThreeWheelConfig ->
		c.leftEncoderName.set("frontLeftMotor")
		c.rightEncoderName.set("backRightMotor")
		c.strafeEncoderName.set("frontRightMotor")
		c.leftPodY.set(6.2787)
		c.rightPodY.set(-5.4827)
		c.strafePodX.set(-7.0814)
		c.forwardTicksToInches.set(0.000733)
		c.strafeTicksToInches.set(0.000730)
		c.turnTicksToRadians.set(0.000062)
		c.leftEncoderDirection.set(Encoder.REVERSE)
		c.rightEncoderDirection.set(Encoder.REVERSE)
		c.strafeEncoderDirection.set(Encoder.FORWARD)
	})
	
	var driveConfig = MecanumConfig(
		Configuration { c: MecanumConfig ->
			c.frontLeftName.set("frontLeftMotor")
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
		Configuration { c: ForesightConfig ->
			val primaryTranslationalForward = Controller.proportional(0.8093316321020193)
			val secondaryTranslationalForward = Controller.proportional(0.29902641232219945)
			val primaryTranslationalLateral = Controller.proportional(3.300377230088197)
			val secondaryTranslationalLateral = Controller.proportional(1.2194011988138247)
			
			c.forwardTranslational.set(
				Controller.piecewise(secondaryTranslationalForward)
					.put(2.5, primaryTranslationalForward)
			)
			c.strafeTranslational.set(
				Controller.piecewise(secondaryTranslationalLateral)
					.put(2.5, primaryTranslationalLateral)
			)
			
			c.coast.set(Controller.proportionalFeedforward(0.0207247585812958))
			c.brake.set(Controller.proportionalFeedforward(0.017616044794101428))
			
			c.headingFeedback.set(Controller.proportional(280.6667463068479))
			c.headingBrakeCoefficients.set(
				Vector2D.cartesian(
					0.26207688808117385,
					-0.05209206292753153
				)
			)
			
			c.linearBrakeCoefficients.set(Matrix.diag(0.07675878817449143, 0.043490226287056635))
			c.quadraticBrakeCoefficients.set(
				Matrix.diag(
					0.0039520303093595124,
					-7.317014613611766E-4
				)
			)
			
			c.maxAchievableForwardVelocity.set(42.17292512445461)
			c.maxAchievableStrafeVelocity.set(35.128650753919516)
			c.naturalForwardDeceleration.set(39.00833891943502)
			c.naturalStrafeDeceleration.set(50.78907689967295)
		}
	)

	fun create(h: HardwareMap): Follower {
		return Follower(
			ThreeWheelLocalizer(h, this.localizerConfig),
			Mecanum(h, this.driveConfig),
			Foresight(this.foresightConfig)
		);
	}
}