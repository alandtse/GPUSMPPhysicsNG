// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/HairTargetDetail.h"

#include <format>

namespace SmpPhysics
{
	std::string FormatHairTargetDetail(const HairTargetDetail& a_detail)
	{
		std::string text;
		if (!a_detail.matchedName.empty()) {
			text += std::format("Matched by {}: {}", a_detail.matchedByWig ? "worn wig" : "hair head part", a_detail.matchedName);
			if (!a_detail.matchedModel.empty())
				text += std::format("\nMesh: {} ({} triangles)", a_detail.matchedModel, a_detail.triangles);
			text += '\n';
		}
		if (a_detail.hairPart.empty())
			text += "Hair head part: none";
		else
			text += std::format("Hair head part: {} ({} extra parts)", a_detail.hairPart, a_detail.extraParts);
		text += "\nWorn on the head: ";
		if (a_detail.worn.empty())
			return text + "nothing";
		for (std::size_t index = 0; index < a_detail.worn.size(); ++index) {
			const WornPiece& piece = a_detail.worn[index];
			const char* slot = piece.hairSlot ? (piece.headSlot ? "hair and head slots" : "hair slot") : "head slot";
			text += std::format("{}{} ({})", index ? ", " : "", piece.name, slot);
		}
		return text;
	}
}
