#pragma once

#include "Math.h"
#include "SmpRig.h"

#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief One rig bone as a cloth particle, at the same index as rig.bones; inverseMass is 0 for a kinematic bone. */
	struct ClothParticle
	{
		std::uint32_t bone{};
		float inverseMass{};
	};

	/** @brief Which rest relation a constraint enforces. */
	enum class ClothConstraintKind : std::uint8_t
	{
		ChainLength,
		Bend,
		Lateral
	};

	/** @brief A distance constraint between two particles and the window its length may take. */
	struct ClothConstraint
	{
		std::uint32_t first{};
		std::uint32_t second{};
		float restLength{};
		float minLength{};
		float maxLength{};
		ClothConstraintKind kind{};
		std::uint32_t batch{};
	};

	/** @brief The cloth particles, their batched constraints and the number of batches. */
	struct ClothGraph
	{
		std::vector<ClothParticle> particles;
		std::vector<ClothConstraint> constraints;
		std::uint32_t batchCount{};
	};

	/** @brief Fraction of a bend constraint's rest length it may compress to, so a chain folds but is not pulled through itself. */
	inline constexpr float kBendMinimumFraction = 0.9f;

	/** @brief Half-window of a lateral joint axis whose lower limit exceeds its upper, which SMP treats as unlimited. */
	inline constexpr float kUnlimitedLateralLength = 1.0e6f;

	/** @brief Whether at least one additional joint joins two non-kinematic bones, so the rig is a lattice rather than a set of chains. */
	[[nodiscard]] bool IsLatticeRig(const SmpRig& a_rig) noexcept;

	/**
	 * @brief One particle per rig bone and the chain, bend and lateral constraints between them, coloured into
	 *        batches that share no particle.
	 * @details A lateral constraint's rest length is the child's rest offset in the parent's frame and its window
	 *          is that length plus or minus the length of the vector of per-axis half-window widths of the joint's
	 *          linear limits, an axis whose lower limit exceeds its upper counting as kUnlimitedLateralLength.
	 */
	[[nodiscard]] ClothGraph BuildClothGraph(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld);
}
