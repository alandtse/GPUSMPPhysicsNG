// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/CrowdWearCall.h"

namespace SmpPhysics
{
	CrowdWearCall CrowdWearCommand(TargetItem a_item, bool a_takeOff, int a_index)
	{
		switch (a_item) {
		case TargetItem::Armor:
			return a_takeOff ? CrowdWearCall{ "stripTestCrowd", std::nullopt, false } : CrowdWearCall{ "setTestCrowdArmor", a_index, std::nullopt };
		case TargetItem::Cloak:
			return { "setTestCrowdCloak", a_takeOff ? -1 : a_index, std::nullopt };
		case TargetItem::Wig:
			return a_takeOff ? CrowdWearCall{ "takeOffTestCrowdWig", std::nullopt, std::nullopt } : CrowdWearCall{ "setTestCrowdWig", a_index, std::nullopt };
		}
		return {};
	}
}
