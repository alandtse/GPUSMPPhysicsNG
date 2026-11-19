// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/HairTargetPick.h"

namespace SmpPhysics
{
	void UpdatePick(HairTargetPick& a_pick, const HairTargetSamples& a_samples)
	{
		if (a_samples.console != a_pick.console) {
			a_pick.console = a_samples.console;
			if (!a_pick.pinned && a_samples.console) {
				a_pick.source = HairTargetSource::Console;
				a_pick.formID = a_samples.console;
			} else if (!a_pick.pinned && a_pick.source == HairTargetSource::Console) {
				a_pick.source = HairTargetSource::Player;
				a_pick.formID = 0;
			}
		}
		if (!a_samples.menuOpen && a_samples.crosshair)
			a_pick.crosshair = a_samples.crosshair;
	}

	bool PinPick(HairTargetPick& a_pick, HairTargetSource a_source)
	{
		std::uint32_t formID = 0;
		if (a_source == HairTargetSource::Console)
			formID = a_pick.console;
		else if (a_source == HairTargetSource::Crosshair)
			formID = a_pick.crosshair;
		if (a_source != HairTargetSource::Player && !formID)
			return false;
		a_pick.source = a_source;
		a_pick.formID = formID;
		a_pick.pinned = true;
		return true;
	}

	void UnpinPick(HairTargetPick& a_pick)
	{
		a_pick.pinned = false;
		if (a_pick.console) {
			a_pick.source = HairTargetSource::Console;
			a_pick.formID = a_pick.console;
		}
	}

	void ForgetPick(HairTargetPick& a_pick, std::uint32_t a_formID)
	{
		if (!a_formID)
			return;
		if (a_pick.console == a_formID)
			a_pick.console = 0;
		if (a_pick.crosshair == a_formID)
			a_pick.crosshair = 0;
		if (a_pick.formID == a_formID) {
			a_pick.source = HairTargetSource::Player;
			a_pick.formID = 0;
			a_pick.pinned = false;
		}
	}
}
