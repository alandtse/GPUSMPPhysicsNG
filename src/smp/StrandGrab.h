#pragma once

#include "Math.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace SmpPhysics
{
	/** @brief How far, in game units, a hair bone may be from the hand's palm-to-fingertip line and still be caught. */
	inline constexpr float kGrabReach = 5.0f;
	/** @brief How far a strand may be from that line and still keep a grip press from drawing the weapon; wider than the reach so a hand closing on hair is covered. */
	inline constexpr float kWeaponGateReach = 14.0f;

	/** @brief Game units in a metre, which HIGGS's selection distances are given in. */
	inline constexpr float kUnitsPerMetre = 69.99125f;
	/** @brief Radius of the sphere HIGGS casts from the palm to pick what a hand will grab (its NearCastRadius default, 5 cm). */
	inline constexpr float kHiggsSelectRadius = 0.05f * kUnitsPerMetre;
	/** @brief How far HIGGS casts that sphere along the palm normal (its NearCastDistance default, 10 cm). */
	inline constexpr float kHiggsSelectDistance = 0.1f * kUnitsPerMetre;
	/** @brief HIGGS's palm normal in hand space, mirrored in x for the left hand (its PalmVector default). */
	inline constexpr Vec3 kHiggsPalmVector{ -0.018f, -0.965f, 0.261f };
	/** @brief Where HIGGS's cast starts, in hand space and game units, mirrored in x for the left hand (its PalmPosition default). */
	inline constexpr Vec3 kHiggsPalmPosition{ 0.0f, -2.4f, 6.0f };
	/** @brief How much closer than a body part a strand may be reached, because hair rests on the surface of the head and body capsules, in game units. */
	inline constexpr float kHairOverBodyMargin = 4.0f;

	/**
	 * @brief Whether a segment lies within what HIGGS would pick from the palm, and how far out along the palm normal.
	 * @param a_palm Where HIGGS's cast starts.
	 * @param a_normal The palm normal, a unit vector.
	 * @param a_extraRadius Added to the cast radius, for a target that is itself thick, such as a body capsule.
	 * @return The distance from the palm, along the normal, to the nearest point of the segment to the cast axis; none when the segment is outside the cast.
	 */
	[[nodiscard]] std::optional<float> PalmCastHit(const Vec3& a_palm, const Vec3& a_normal, const Vec3& a_start, const Vec3& a_end, float a_extraRadius = 0.0f) noexcept;

	/** @brief What a palm is pointed at. */
	enum class PalmTarget
	{
		None,  ///< Nothing a grip would take.
		Hair,  ///< A strand, nearer than any body part: the grip is meant for it.
		Other  ///< A body part is as near or nearer, so the grip is left to HIGGS.
	};

	/**
	 * @brief Whether a palm is pointed at hair, something else or nothing.
	 * @param a_hairAlong PalmCastHit of the nearest strand, or none.
	 * @param a_otherAlong PalmCastHit of the nearest body part of another actor, or none.
	 */
	[[nodiscard]] PalmTarget DecidePalmTarget(std::optional<float> a_hairAlong, std::optional<float> a_otherAlong) noexcept;

	/** @brief What to tell HIGGS about one hand this frame. */
	enum class HiggsHandAction
	{
		None,
		Disable,  ///< Stop HIGGS picking things up with the hand, so it leaves the hair to this feature.
		Enable    ///< Give the hand back to HIGGS.
	};

	/**
	 * @brief Whether HIGGS should be kept from picking things up with a hand.
	 * @param a_onHair The hand holds a strand or has one within grab reach.
	 * @param a_holdingObject HIGGS already holds an object in the hand, which is left alone.
	 * @param a_disabled This feature has already disabled the hand.
	 */
	[[nodiscard]] HiggsHandAction DecideHiggsHand(bool a_onHair, bool a_holdingObject, bool a_disabled) noexcept;

	/** @brief Controller pulse when a hand closes on hair, in the units BSOpenVR takes (4000 microseconds each). */
	inline constexpr float kGrabHapticDuration = 12.0f;
	/** @brief A held strand pulled less than this far from the hand gives no pulse, in game units. */
	inline constexpr float kHapticMinimumTension = 1.0f;
	/** @brief Pulse length per game unit of pull on a held strand. */
	inline constexpr float kHapticDurationPerUnit = 0.5f;
	/** @brief Longest pulse while a strand is held. */
	inline constexpr float kHapticMaxDuration = 8.0f;
	/** @brief Least time between pulses while a strand is held, in seconds. */
	inline constexpr float kHapticIntervalSeconds = 0.1f;

	/**
	 * @brief The controller pulse a hand gets this frame, or 0 for none.
	 * @param a_justGrabbed A strand was taken this frame.
	 * @param a_tension How far the held strands are pulled from the hand, in game units.
	 * @param a_secondsSinceLastPulse Time since this hand last pulsed.
	 */
	[[nodiscard]] float HapticPulseDuration(bool a_justGrabbed, float a_tension, float a_secondsSinceLastPulse) noexcept;

	/** @brief How far a held point may be from where the hand wants it before the strand has slipped from the hand, in game units. */
	inline constexpr float kGrabDropDistance = 14.0f;
	/** @brief How long a held strand may stay beyond kGrabDropDistance before it is dropped, in seconds. */
	inline constexpr float kGrabDropSeconds = 0.15f;

	/**
	 * @brief Whether a held strand can no longer follow the hand and should be let go.
	 * @param a_tension The largest distance between a held point and where the hand wants it, in game units.
	 * @param a_seconds How long the strand has been beyond kGrabDropDistance; updated here, and reset while it is not.
	 */
	[[nodiscard]] bool ShouldDropGrab(float a_tension, float a_frameSeconds, float& a_seconds) noexcept;

	/** @brief How far out from the palm, along its normal, a closing hand's curl catches hair, in game units. */
	inline constexpr float kGrabCurlDepth = 3.5f;
	/** @brief How far along the open fingers hair is caught, in game units. */
	inline constexpr float kGrabFingerLength = 6.0f;
	/** @brief How far the finger line sits off the palm plane toward the inside of the fingers, in game units. */
	inline constexpr float kGrabFingerLift = 1.0f;
	/** @brief How far behind the palm plane hair still counts, in game units: the thickness of the fingers, so hair clipping through them is caught. */
	inline constexpr float kGrabFingerThickness = 1.0f;
	/** @brief How far past the fingertips hair still counts, in game units. */
	inline constexpr float kGrabTipSlack = 1.0f;
	/** @brief How far back toward the wrist from the palm point hair still counts, in game units. */
	inline constexpr float kGrabWristSlack = 1.0f;

	/** @brief The region of a hand that catches hair: up to two segments, and with palmSideOnly the slab of space the palm side and fingers cover. */
	struct GrabHandShape
	{
		std::array<std::array<Vec3, 2>, 2> segments{};  ///< Segment start and end, in world space.
		std::size_t segmentCount = 0;
		Vec3 palm{};
		Vec3 normal{};              ///< Unit palm normal; the inside of the hand is the side it points to.
		Vec3 fingerDirection{};     ///< Unit vector along the open fingers.
		bool palmSideOnly = false;  ///< Only hair in front of the palm plane (less kGrabFingerThickness) and between the wrist and the fingertips counts.
	};

	/**
	 * @brief The region the inside of a hand catches hair in: the curl between the palm and the closing fingers, and the fingers themselves.
	 * @details A hand slides its fingers between the hair and the head and closes, so hair along the fingers or already in the curl is taken.
	 *          Only the palm side collects hair: hair behind the hand, or beyond the fingertips, never counts, and hair that clips the fingers does.
	 * @param a_palm Where the palm is.
	 * @param a_normal Unit vector out of the palm.
	 * @param a_fingerDirection Unit vector along the open fingers.
	 */
	[[nodiscard]] GrabHandShape MakeHandGrabShape(const Vec3& a_palm, const Vec3& a_normal, const Vec3& a_fingerDirection) noexcept;

	/** @brief How far past the strand's own length from its root the hand may reach before the strand can no longer follow, in game units. */
	inline constexpr float kGrabOverreachSlack = 6.0f;
	/** @brief How long a hand may stay past that before the strand is dropped, in seconds. */
	inline constexpr float kGrabOverreachSeconds = 0.1f;

	/**
	 * @brief Whether a strand pulled taut by a hand beyond its own length should be let go.
	 * @details A strand cannot stretch, so a hand farther from the strand's root than the strand is long only drags its joints past their limits.
	 * @param a_overreach The hand's distance from the strand's root minus the strand's length, in game units.
	 * @param a_seconds How long the hand has been beyond kGrabOverreachSlack; updated here, and reset while it is not.
	 */
	[[nodiscard]] bool ShouldDropOverreach(float a_overreach, float a_frameSeconds, float& a_seconds) noexcept;

	/** @brief Most strands one hand holds at once. */
	inline constexpr std::size_t kMaxGrabPinsPerHand = 6;

	/** @brief A stretch of hair a hand could catch: the part of a bone's strand between its origin and the next bone's. */
	struct GrabCandidate
	{
		std::uint64_t claim{};  ///< Token of the claim the bone belongs to.
		std::uint32_t group{};  ///< Stretches of one claim that share a group belong to one strand, so a hand holds only the nearest of them.
		int bone{ -1 };         ///< Rig bone within the claim.
		Vec3 start{};           ///< World position of the strand's start.
		Vec3 end{};             ///< World position of its end.
	};

	/** @brief What the hand catches: which candidate, and the point of its strand nearest the hand. */
	struct GrabPick
	{
		std::size_t index{};
		Vec3 point{};
		float distance{};
	};

	/**
	 * @brief The strands inside the grip: the nearest stretch of each strand within a_reach of the hand's shape, limited to the palm side when the shape says so.
	 * @details Nearest first, then by claim token and bone, so the result does not depend on the order candidates are listed. A strand is
	 *          a group of one claim; a hand closing on a lock of hair therefore catches each strand once, not each segment of it.
	 * @return At most a_maxCount picks; empty when nothing is in reach or a point is not finite.
	 */
	[[nodiscard]] std::vector<GrabPick> PickGrabCandidates(std::span<const GrabCandidate> a_candidates, const GrabHandShape& a_hand, float a_reach, std::size_t a_maxCount);
}
