// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/PhysicsHealth.h"

#include "smp/RepairLog.h"

namespace SmpPhysics
{
	Health Worse(Health a_left, Health a_right)
	{
		return static_cast<int>(a_left) >= static_cast<int>(a_right) ? a_left : a_right;
	}

	Health ClassifyFile(bool a_parsedOk, std::string_view a_parseReason, bool a_repaired, std::string_view a_declineReason)
	{
		if (!a_parsedOk && RepairLog::IsFileFault(a_parseReason))
			return Health::Broken;
		if (!a_declineReason.empty() && RepairLog::IsFileFault(a_declineReason))
			return Health::Broken;
		return a_repaired ? Health::Repaired : Health::Working;
	}

	Health ClassifyItem(std::span<const FileHealth> a_files)
	{
		Health worst = Health::None;
		bool anyMissing = false;
		bool anyPresent = false;
		for (const FileHealth& file : a_files) {
			if (file.missing) {
				anyMissing = true;
				continue;
			}
			anyPresent = true;
			worst = Worse(worst, file.health);
		}
		return anyMissing && !anyPresent ? Health::Broken : worst;
	}
}
