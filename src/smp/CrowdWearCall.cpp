// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/CrowdWearCall.h"

namespace SmpPhysics
{
	CrowdWearCall CrowdWearCommand(TargetItem a_item, bool a_takeOff, int a_index)
	{
		using namespace CrowdWearCommands;
		switch (a_item) {
		case TargetItem::Armor:
			return a_takeOff ? CrowdWearCall{ kStrip, std::nullopt, false } : CrowdWearCall{ kArmor, a_index, std::nullopt };
		case TargetItem::Cloak:
			return { kCloak, a_takeOff ? -1 : a_index, std::nullopt };
		case TargetItem::Wig:
			return a_takeOff ? CrowdWearCall{ kTakeOffWig, std::nullopt, std::nullopt } : CrowdWearCall{ kWig, a_index, std::nullopt };
		}
		return {};
	}
}
