// Wind drag on a hair segment. The drag and gust constants are defined here.
#ifndef __HAIR_WIND_DEPENDENCY_HLSL__
#define __HAIR_WIND_DEPENDENCY_HLSL__

namespace HairWind
{
	/** @brief The force of wind drag on a bone's hair segment. */
	float3 WindForce(float3 a_wind, float a_windTime, float a_area, float a_mass, float a_boneFactor, float3 a_axis, float3 a_origin, float3 a_velocity)
	{
		static const float kBulletPerSkyrimUnit = 0.01425f;
		static const float kMaxGustTime = 4096.0f;
		static const float kEpsilon = 1.1920929e-7f;
		static const float kWindDrag = 1.0e-6f;
		static const float kMaxWindAcceleration = 400.0f;
		static const float kGustSpeedPerAcceleration = 0.05f;
		static const float kMinGustSpeed = 180.0f;
		static const float kMaxGustSpeed = 1200.0f;
		static const float kCrosswindTimeScale = 0.004f;
		static const float kVerticalPhaseScale = 0.006f;
		static const float kLongGustRate = 0.55f;
		static const float kLongGustVertical = 0.35f;
		static const float kMidGustRate = 1.35f;
		static const float kMidGustFeedback = 0.5f;
		static const float kFlutterRate = 4.15f;
		static const float kFlutterVertical = 2.2f;
		static const float kPulseRate = 0.85f;
		static const float kPulseVertical = 0.6f;
		static const float kSidewaysMid = 0.13f;
		static const float kSidewaysFlutter = 0.06f;
		static const float kSidewaysBurst = 0.04f;
		static const float kLiftRate = 1.9f;
		static const float kLiftVertical = 1.4f;
		static const float kLiftGust = 0.018f;
		static const float kLiftFlutter = 0.01f;

		float magnitude = length(a_wind);
		if (!(magnitude > kEpsilon) || !(a_boneFactor > kEpsilon) || !(a_area > kEpsilon))
			return float3(0.0f, 0.0f, 0.0f);
		float3 relativeWind = a_wind - a_velocity;
		float3 across = relativeWind - a_axis * dot(a_axis, relativeWind);
		float speed = length(across);
		if (!(speed > kEpsilon))
			return float3(0.0f, 0.0f, 0.0f);
		float3 force = across * (kWindDrag * a_area * a_boneFactor * speed);
		float forceLength = length(force);
		float maximumForce = a_mass * kMaxWindAcceleration;
		if (forceLength > maximumForce) {
			force *= maximumForce / forceLength;
			forceLength = maximumForce;
		}

		float time = fmod(a_windTime, kMaxGustTime);
		float3 direction = a_wind / magnitude;
		float3 up = float3(0.0f, 0.0f, 1.0f);
		float3 side = cross(direction, up);
		side = dot(side, side) < kEpsilon ? float3(1.0f, 0.0f, 0.0f) : normalize(side);

		float gustSpeed = clamp(magnitude * kGustSpeedPerAcceleration, kMinGustSpeed, kMaxGustSpeed);
		float3 origin = a_origin * kBulletPerSkyrimUnit;
		float advected = time - dot(origin, direction) / gustSpeed - dot(origin, side) * kCrosswindTimeScale;
		float vertical = origin.z * kVerticalPhaseScale;

		float longGust = sin(advected * kLongGustRate + vertical * kLongGustVertical);
		float midGust = sin(advected * kMidGustRate + vertical + longGust * kMidGustFeedback);
		float flutter = sin(advected * kFlutterRate + vertical * kFlutterVertical + midGust);
		float pulse = 0.5f + 0.5f * sin(advected * kPulseRate + vertical * kPulseVertical);
		float burst = pulse * pulse;

		float sideways = (midGust * kSidewaysMid + flutter * kSidewaysFlutter + burst * kSidewaysBurst) * forceLength;
		float lift = (sin(advected * kLiftRate + vertical * kLiftVertical) * kLiftGust + flutter * kLiftFlutter) * forceLength;
		return force + side * sideways + up * lift;
	}
}

#endif
