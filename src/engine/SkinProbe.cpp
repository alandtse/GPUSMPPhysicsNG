#include "Features/SmpPhysics.h"

#include "Globals.h"
#include "Utils/SehGuard.h"

#include <array>
#include <cstring>

namespace
{
	/** @brief Copies engine-owned memory that may be stale, returning false instead of faulting. */
	bool SafeCopy(const void* a_source, void* a_destination, std::size_t a_bytes)
	{
		return Util::SehGuarded([&] { std::memcpy(a_destination, a_source, a_bytes); });
	}
}

json SmpPhysicsFeature::SkinProbeJson(const json& a_args)
{
	const RE::FormID formID = a_args.value("actorFormID", 0u);
	const std::string meshName = a_args.value("mesh", std::string{});
	const int boneLimit = std::clamp(a_args.value("bones", 3), 1, 8);
	auto* actor = formID ? RE::TESForm::LookupByID<RE::Actor>(formID) : nullptr;
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root || meshName.empty())
		return { { "error", "actorFormID and mesh are required and the actor must be loaded" } };
	RE::NiAVObject* object = root->GetObjectByName(RE::BSFixedString(meshName.c_str()));
	auto* geometry = object ? object->AsGeometry() : nullptr;
	RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
	if (!skin || !skin->skinData || !skin->bones)
		return { { "error", "the mesh has no skin instance" } };

	json result = {
		{ "numMatrices", skin->numMatrices },
		{ "numRegisters", skin->numRegisters },
		{ "allocatedSize", skin->allocatedSize },
		{ "frameID", skin->frameID },
		{ "boneMatricesPointer", reinterpret_cast<std::uintptr_t>(skin->boneMatrices) },
		{ "prevBoneMatricesPointer", reinterpret_cast<std::uintptr_t>(skin->prevBoneMatrices) },
		{ "boneCountInSkinData", skin->skinData->GetBoneCount() }
	};
	if (skin->rootParent) {
		result["rootParentName"] = skin->rootParent->name.c_str() ? skin->rootParent->name.c_str() : "";
		result["rootParentWorld"] = { skin->rootParent->world.translate.x, skin->rootParent->world.translate.y, skin->rootParent->world.translate.z };
	}
	result["actorRootWorld"] = { root->world.translate.x, root->world.translate.y, root->world.translate.z };
	if (const auto* camera = globals::game::playerCamera) {
		const auto& position = camera->GetRuntimeData2().pos;
		result["cameraPosition"] = { position.x, position.y, position.z };
	}

	json bones = json::array();
	const std::uint32_t count = std::min<std::uint32_t>(skin->numMatrices, static_cast<std::uint32_t>(boneLimit));
	for (std::uint32_t index = 0; index < count; ++index) {
		const RE::NiAVObject* bone = skin->bones[index];
		json entry = { { "index", index }, { "name", bone && bone->name.c_str() ? bone->name.c_str() : "" } };
		const RE::NiTransform skinToBone = skin->skinData->GetBoneDataSkinToBone(index);
		if (bone) {
			const std::array<float, 12> expected = SmpPhysics::ExpectedSkinRows(bone->world, skinToBone);
			entry["expected"] = expected;
			entry["nodeWorldTranslation"] = { bone->world.translate.x, bone->world.translate.y, bone->world.translate.z };
		}
		const auto readRows = [index](const void* a_base) -> json {
			if (!a_base)
				return nullptr;
			std::array<float, 12> rows{};
			if (!SafeCopy(static_cast<const std::uint8_t*>(a_base) + static_cast<std::size_t>(index) * sizeof(rows), rows.data(), sizeof(rows)))
				return "unreadable";
			return rows;
		};
		entry["engineRows"] = readRows(skin->boneMatrices);
		entry["enginePreviousRows"] = readRows(skin->prevBoneMatrices);
		bones.push_back(std::move(entry));
	}
	result["bones"] = std::move(bones);
	return result;
}

json SmpPhysicsFeature::KinematicCheckJson(const json& a_args) const
{
	const RE::FormID formID = a_args.value("actorFormID", 0u);
	const std::string meshName = a_args.value("mesh", std::string{});
	auto* actor = formID ? RE::TESForm::LookupByID<RE::Actor>(formID) : nullptr;
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root || meshName.empty())
		return { { "error", "actorFormID and mesh are required and the actor must be loaded" } };
	RE::NiAVObject* object = root->GetObjectByName(RE::BSFixedString(meshName.c_str()));
	auto* geometry = object ? object->AsGeometry() : nullptr;
	RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
	if (!skin)
		return { { "error", "the mesh has no skin instance" } };
	if (a_args.value("arm", false))
		return { { "armed", gpuSkinning.ArmKinematicCheck(skin) } };

	const SmpPhysics::GpuSkinning::KinematicCheck check = gpuSkinning.GetKinematicCheck(skin);
	return { { "armed", check.armed }, { "samples", check.samples }, { "staticBones", check.staticBones },
		{ "engineVsBindPoseMax", check.engineVsBindPoseMax }, { "engineVsStepPoseMax", check.engineVsStepPoseMax },
		{ "stepVsBindTranslationMax", check.stepVsBindTranslationMax } };
}

json SmpPhysicsFeature::SkinPaletteCheckJson(const json& a_args) const
{
	const RE::FormID formID = a_args.value("actorFormID", 0u);
	const std::string meshName = a_args.value("mesh", std::string{});
	auto* actor = formID ? RE::TESForm::LookupByID<RE::Actor>(formID) : nullptr;
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root || meshName.empty())
		return { { "error", "actorFormID and mesh are required and the actor must be loaded" } };
	RE::NiAVObject* object = root->GetObjectByName(RE::BSFixedString(meshName.c_str()));
	auto* geometry = object ? object->AsGeometry() : nullptr;
	RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
	if (!skin)
		return { { "error", "the mesh has no skin instance" } };

	const SmpPhysics::GpuSkinning::Capture capture = gpuSkinning.RequestCapture(skin);
	json result = { { "registeredSkins", gpuSkinning.SkinCount() }, { "captured", capture.valid } };
	if (!capture.valid)
		return result;

	const auto translationDistance = [](const SmpPhysics::SkinRows& a_left, const SmpPhysics::SkinRows& a_right) {
		const float dx = a_left[3] - a_right[3];
		const float dy = a_left[7] - a_right[7];
		const float dz = a_left[11] - a_right[11];
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	};
	const auto largestRowDifference = [](const SmpPhysics::SkinRows& a_left, const SmpPhysics::SkinRows& a_right) {
		float largest = 0.0f;
		for (std::size_t index = 0; index < a_left.size(); ++index)
			largest = std::max(largest, std::abs(a_left[index] - a_right[index]));
		return largest;
	};

	std::uint32_t staticBones = 0;
	std::uint32_t dynamicBones = 0;
	float staticCurrentMax = 0.0f;
	float staticPreviousMax = 0.0f;
	float dynamicTranslationMax = 0.0f;
	float dynamicRowMax = 0.0f;
	float dynamicMotionMax = 0.0f;
	float shaderVersusReferenceMax = 0.0f;
	int firstDynamic = -1;
	for (std::uint32_t index = 0; index < capture.boneCount; ++index) {
		if (capture.slots[index] == SmpPhysics::GpuSkinning::kStaticBone) {
			++staticBones;
			staticCurrentMax = std::max(staticCurrentMax, largestRowDifference(capture.current[index], capture.engineCurrent[index]));
			staticPreviousMax = std::max(staticPreviousMax, largestRowDifference(capture.previous[index], capture.enginePrevious[index]));
		} else {
			++dynamicBones;
			if (firstDynamic < 0)
				firstDynamic = static_cast<int>(index);
			dynamicTranslationMax = std::max(dynamicTranslationMax, translationDistance(capture.current[index], capture.engineCurrent[index]));
			dynamicRowMax = std::max(dynamicRowMax, largestRowDifference(capture.current[index], capture.engineCurrent[index]));
			dynamicMotionMax = std::max(dynamicMotionMax, translationDistance(capture.current[index], capture.previous[index]));
			shaderVersusReferenceMax = std::max(shaderVersusReferenceMax, largestRowDifference(capture.current[index], capture.reference[index]));
		}
	}
	result["boneCount"] = capture.boneCount;
	result["staticBones"] = staticBones;
	result["dynamicBones"] = dynamicBones;
	result["staticCurrentMaxRowDifference"] = staticCurrentMax;
	result["staticPreviousMaxRowDifference"] = staticPreviousMax;
	result["dynamicTranslationDifferenceMax"] = dynamicTranslationMax;
	result["dynamicRowDifferenceMax"] = dynamicRowMax;
	result["dynamicCurrentToPreviousTranslationMax"] = dynamicMotionMax;
	result["dynamicShaderVersusCpuReferenceMaxRowDifference"] = shaderVersusReferenceMax;
	result["batched"] = !capture.batchCurrent.empty();
	if (!capture.batchCurrent.empty()) {
		float batchMax = 0.0f;
		for (std::uint32_t index = 0; index < capture.boneCount; ++index)
			batchMax = std::max(batchMax, largestRowDifference(capture.batchCurrent[index], capture.current[index]));
		result["batchedCurrentVsPerDrawMaxRowDifference"] = batchMax;
	}
	if (firstDynamic >= 0) {
		result["firstDynamicBone"] = { { "index", firstDynamic }, { "bound", capture.current[static_cast<std::size_t>(firstDynamic)] },
			{ "engine", capture.engineCurrent[static_cast<std::size_t>(firstDynamic)] } };
	}
	return result;
}
