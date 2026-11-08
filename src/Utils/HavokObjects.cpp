#include "HavokObjects.h"

#include "Globals.h"
#include "Utils/ActorUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

namespace Util::HavokObjects
{
	namespace
	{
		constexpr float kCandidateRadius = 1500.0f;
		constexpr std::chrono::milliseconds kCandidateRefresh{ 250 };
		constexpr float kMinCapsuleRadius = 1.0f;

		std::mutex candidateMutex;
		std::vector<RE::FormID> candidates;
		std::chrono::steady_clock::time_point candidatesTaken{};

		/** @brief The rigid body that carries a reference's 3D, searching its node tree for the first collision object. */
		RE::hkpRigidBody* FindRigidBody(RE::NiAVObject* a_root)
		{
			if (!a_root)
				return nullptr;
			if (auto* collision = a_root->GetCollisionObject()) {
				if (auto* body = collision->GetRigidBody())
					return body->GetRigidBody();
			}
			if (auto* node = a_root->AsNode()) {
				for (auto& child : node->GetChildren()) {
					if (auto* found = FindRigidBody(child.get()))
						return found;
				}
			}
			return nullptr;
		}

		bool IsLooseObject(RE::TESObjectREFR* a_reference)
		{
			if (!a_reference || a_reference->As<RE::Actor>() || !a_reference->Is3DLoaded())
				return false;
			const auto* body = FindRigidBody(a_reference->Get3D());
			if (!body)
				return false;
			const auto motion = body->motion.type.get();
			return motion != RE::hkpMotion::MotionType::kInvalid && motion != RE::hkpMotion::MotionType::kFixed &&
			       motion != RE::hkpMotion::MotionType::kCharacter;
		}

		void RefreshCandidates()
		{
			const auto now = std::chrono::steady_clock::now();
			const std::scoped_lock lock(candidateMutex);
			if (now - candidatesTaken < kCandidateRefresh)
				return;
			candidatesTaken = now;
			candidates.clear();
			const auto* player = globals::game::player;
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!cell)
				return;
			cell->ForEachReferenceInRange(player->GetPosition(), kCandidateRadius, [](RE::TESObjectREFR* a_reference) {
				if (IsLooseObject(a_reference))
					candidates.push_back(a_reference->GetFormID());
				return RE::BSContainer::ForEachResult::kContinue;
			});
		}

		/** @brief a_reference's bound box as a capsule along its longest axis; false when the object has no usable box. */
		bool ToCapsule(RE::TESObjectREFR* a_reference, BodyCapsules::Capsule& a_capsule)
		{
			auto* object = a_reference->Get3D();
			if (!object)
				return false;
			const RE::NiPoint3 low = a_reference->GetBoundMin();
			const RE::NiPoint3 high = a_reference->GetBoundMax();
			const float scale = object->world.scale;
			const RE::NiPoint3 half{ std::fabs(high.x - low.x) * 0.5f * scale, std::fabs(high.y - low.y) * 0.5f * scale, std::fabs(high.z - low.z) * 0.5f * scale };
			const RE::NiPoint3 localCentre{ (high.x + low.x) * 0.5f * scale, (high.y + low.y) * 0.5f * scale, (high.z + low.z) * 0.5f * scale };
			if (!(half.x > 0.0f) || !(half.y > 0.0f) || !(half.z > 0.0f) || !std::isfinite(half.x + half.y + half.z))
				return false;

			const float extents[3] = { half.x, half.y, half.z };
			const int longest = extents[0] >= extents[1] && extents[0] >= extents[2] ? 0 : (extents[1] >= extents[2] ? 1 : 2);
			float radius = 0.0f;
			for (int axis = 0; axis < 3; ++axis) {
				if (axis != longest)
					radius = std::max(radius, extents[axis]);
			}
			radius = std::max(radius, kMinCapsuleRadius);
			const float boundRadius = object->worldBound.radius;
			if (boundRadius > 0.0f)
				radius = std::min(radius, boundRadius);
			const float halfLength = std::max(extents[longest] - radius, 0.0f);

			const RE::NiPoint3 centre = object->world.translate + object->world.rotate * localCentre;
			RE::NiPoint3 axis{};
			(&axis.x)[longest] = 1.0f;
			const RE::NiPoint3 along = object->world.rotate * axis * halfLength;
			a_capsule.pointA = ToFloat3(centre - along);
			a_capsule.pointB = ToFloat3(centre + along);
			a_capsule.radius = radius;
			a_capsule.owner = a_reference->GetFormID();
			return true;
		}
	}

	std::vector<BodyCapsules::Capsule> Near(const RE::NiPoint3& a_centre, float a_radius, float a_reach)
	{
		RefreshCandidates();
		std::vector<RE::FormID> ids;
		{
			const std::scoped_lock lock(candidateMutex);
			ids = candidates;
		}
		std::vector<std::pair<float, BodyCapsules::Capsule>> ranked;
		for (const RE::FormID id : ids) {
			auto* reference = RE::TESForm::LookupByID<RE::TESObjectREFR>(id);
			if (!reference || !reference->Is3DLoaded())
				continue;
			BodyCapsules::Capsule capsule;
			if (!ToCapsule(reference, capsule))
				continue;
			const float3 mid = (capsule.pointA + capsule.pointB) * 0.5f;
			const float3 offset = mid - ToFloat3(a_centre);
			const float gap = offset.Length() - (capsule.pointB - capsule.pointA).Length() * 0.5f - capsule.radius - a_radius;
			if (gap > a_reach)
				continue;
			ranked.emplace_back(gap, capsule);
		}
		std::ranges::sort(ranked, [](const auto& a_left, const auto& a_right) { return a_left.first < a_right.first; });
		if (ranked.size() > MaxObjectCapsules)
			ranked.resize(MaxObjectCapsules);
		std::vector<BodyCapsules::Capsule> result;
		result.reserve(ranked.size());
		for (const auto& entry : ranked)
			result.push_back(entry.second);
		return result;
	}
}
