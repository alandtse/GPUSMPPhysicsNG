// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/CollisionOverride.h"

#include <algorithm>

namespace SmpPhysics
{
	std::size_t ApplyCollisionOverrides(const std::vector<CollisionOverride>& a_overrides, const std::vector<std::string>& a_boneShapeName,
		const std::vector<std::string>& a_surfaceNames, std::vector<std::vector<BoneSphere>>& a_spheres, std::vector<std::uint32_t>& a_boneSurfaceMask)
	{
		std::size_t changed = 0;
		for (const CollisionOverride& override : a_overrides) {
			for (std::size_t bone = 0; bone < a_boneShapeName.size() && bone < a_spheres.size(); ++bone) {
				if (a_boneShapeName[bone] != override.piece)
					continue;
				if (override.target == kOverrideColliders) {
					for (BoneSphere& sphere : a_spheres[bone])
						sphere.exemptMask = 0xFFFFFFFFu;
					++changed;
					continue;
				}
				const auto surface = std::ranges::find(a_surfaceNames, override.target);
				if (surface == a_surfaceNames.end() || static_cast<std::size_t>(surface - a_surfaceNames.begin()) >= 32)
					continue;
				if (a_boneSurfaceMask.size() < a_boneShapeName.size())
					a_boneSurfaceMask.resize(a_boneShapeName.size(), 0xFFFFFFFFu);
				a_boneSurfaceMask[bone] &= ~(1u << (surface - a_surfaceNames.begin()));
				++changed;
			}
		}
		return changed;
	}
}
