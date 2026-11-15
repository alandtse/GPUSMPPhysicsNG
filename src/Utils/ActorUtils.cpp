#include "ActorUtils.h"

#include "Globals.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <span>
#include <string_view>

namespace
{
	RE::NiPoint3 TransformHavokPoint(const RE::hkTransform& transform, const RE::hkVector4& point)
	{
		alignas(16) float localPoint[4];
		_mm_store_ps(localPoint, point.quad);

		__m128 worldPoint = transform.translation.quad;
		worldPoint = _mm_add_ps(worldPoint, _mm_mul_ps(transform.rotation.col0.quad, _mm_set1_ps(localPoint[0])));
		worldPoint = _mm_add_ps(worldPoint, _mm_mul_ps(transform.rotation.col1.quad, _mm_set1_ps(localPoint[1])));
		worldPoint = _mm_add_ps(worldPoint, _mm_mul_ps(transform.rotation.col2.quad, _mm_set1_ps(localPoint[2])));

		alignas(16) float transformedPoint[4];
		_mm_store_ps(transformedPoint, worldPoint);
		return RE::NiPoint3(transformedPoint[0], transformedPoint[1], transformedPoint[2]) *
		       RE::bhkWorld::GetWorldScaleInverse();
	}

	bool IsCapsuleFinite(const Util::ShapeCollisionCapsule& capsule)
	{
		return std::isfinite(capsule.pointA.x) && std::isfinite(capsule.pointA.y) && std::isfinite(capsule.pointA.z) &&
		       std::isfinite(capsule.pointB.x) && std::isfinite(capsule.pointB.y) && std::isfinite(capsule.pointB.z) &&
		       std::isfinite(capsule.radius) && capsule.radius > 0.0f;
	}

	/** @brief The world-space capsule of a Havok capsule shape on a body with a_transform. */
	bool WorldCapsule(const RE::hkpCapsuleShape& a_shape, const RE::hkTransform& a_transform, Util::ShapeCollisionCapsule& a_capsule)
	{
		a_capsule.pointA = TransformHavokPoint(a_transform, a_shape.vertexA);
		a_capsule.pointB = TransformHavokPoint(a_transform, a_shape.vertexB);
		a_capsule.radius = a_shape.radius * RE::bhkWorld::GetWorldScaleInverse();
		return IsCapsuleFinite(a_capsule);
	}

	/** @brief Ray hit filter that drops one reference's own collision. */
	class OccluderRayCollector : public RE::hkpClosestRayHitCollector
	{
	public:
		const RE::TESObjectREFR* alsoIgnored{};

		void AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo) override
		{
			const RE::hkpCdBody* body = std::addressof(a_body);
			for (const auto* parent = body->parent; parent; parent = parent->parent)
				body = parent;
			if (!body)
				return;
			const auto* owner = RE::TESHavokUtilities::FindCollidableRef(*static_cast<const RE::hkpCollidable*>(body));
			if (alsoIgnored && owner == alsoIgnored)
				return;
			RE::hkpClosestRayHitCollector::AddRayHit(a_body, a_hitInfo);
		}
	};
}

namespace Util
{
	// Actor query pattern adapted from po3 under MIT.
	// https://github.com/powerof3/PapyrusExtenderSSE/blob/7a73b47bc87331bec4e16f5f42f2dbc98b66c3a7/include/Papyrus/Functions/Faction.h#L24C7-L46
	void ForEachLoadedActor(const std::function<void(RE::Actor*)>& a_callback)
	{
		if (!a_callback)
			return;

		auto* player = globals::game::player;
		if (player)
			a_callback(player);

		if (const auto* processLists = RE::ProcessLists::GetSingleton()) {
			for (const auto& actorHandle : processLists->highActorHandles) {
				if (const auto actor = actorHandle.get(); actor && actor.get() != player)
					a_callback(actor.get());
			}
		}
	}

	std::optional<OccluderHit> FirstOccluder(RE::Actor& a_actor, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to)
	{
		auto* cell = a_actor.GetParentCell();
		auto* world = cell ? cell->GetbhkWorld() : nullptr;
		if (!world)
			return std::nullopt;
		const float scale = RE::bhkWorld::GetWorldScale();
		RE::bhkPickData pickData{};
		pickData.rayInput.from = a_from * scale;
		pickData.rayInput.to = a_to * scale;
		pickData.rayInput.enableShapeCollectionFilter = false;
		pickData.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kLOS);
		OccluderRayCollector collector;
		collector.alsoIgnored = std::addressof(a_actor);
		pickData.closestRayHitCollector = &collector;
		if (!world->PickObject(pickData))
			return std::nullopt;
		const auto* collidable = pickData.rayOutput.rootCollidable;
		const auto* owner = collidable ? RE::TESHavokUtilities::FindCollidableRef(*collidable) : nullptr;
		return OccluderHit{ pickData.rayOutput.hitFraction * (a_to - a_from).Length(), owner ? owner->GetFormID() : 0u };
	}

	void ForEachGeometry(RE::NiAVObject* a_root, const std::function<void(RE::BSGeometry*)>& a_callback)
	{
		if (!a_root || !a_callback)
			return;

		RE::BSVisit::TraverseScenegraphGeometries(a_root, [&a_callback](RE::BSGeometry* a_geometry) {
			a_callback(a_geometry);
			return RE::BSVisit::BSVisitControl::kContinue;
		});
	}

	void ForEachActorGeometry(RE::Actor* a_actor, const std::function<void(RE::BSGeometry*)>& a_callback)
	{
		if (!a_actor || !a_actor->Is3DLoaded() || !a_callback)
			return;

		auto* thirdPersonRoot = a_actor->Get3D(false);
		auto* firstPersonRoot = a_actor->Get3D(true);
		ForEachGeometry(thirdPersonRoot, a_callback);
		if (firstPersonRoot != thirdPersonRoot)
			ForEachGeometry(firstPersonRoot, a_callback);
	}

	bool GetShapeCollisionCapsule(RE::bhkNiCollisionObject* collisionObj, ShapeCollisionCapsule& capsule)
	{
		if (!collisionObj)
			return false;

		RE::bhkRigidBody* bhkRigid = collisionObj->body.get() ? collisionObj->body.get()->AsBhkRigidBody() : nullptr;
		RE::hkpRigidBody* hkpRigid = bhkRigid ? skyrim_cast<RE::hkpRigidBody*>(bhkRigid->referencedObject.get()) : nullptr;
		const auto* shape = hkpRigid ? hkpRigid->collidable.GetShape() : nullptr;
		if (bhkRigid && hkpRigid && shape && !skyrim_cast<const RE::hkpListShape*>(shape)) {  // Ignore hkpListShape, unsupported
			if (shape->type == RE::hkpShapeType::kCapsule) {
				RE::hkTransform transform;
				bhkRigid->GetTransform(transform);
				return WorldCapsule(*static_cast<const RE::hkpCapsuleShape*>(shape), transform, capsule);
			}

			RE::hkVector4 massCenter;
			bhkRigid->GetCenterOfMassWorld(massCenter);
			float massTrans[4];
			// Use unaligned store to avoid UB from potential stack misalignment
			_mm_storeu_ps(massTrans, massCenter.quad);
			capsule.pointA = RE::NiPoint3(massTrans[0], massTrans[1], massTrans[2]) * RE::bhkWorld::GetWorldScaleInverse();
			capsule.pointB = capsule.pointA;
			return Util::ExtractShapeBound(shape, capsule.radius) && IsCapsuleFinite(capsule);
		}
		return false;
	}

	namespace
	{
		/** @brief A world-space capsule along the longest side of an axis-aligned box in the body's own frame, with the mean of the other two half sides as its radius. */
		bool BoxCapsule(const float a_centre[3], const float a_half[3], const RE::hkTransform& a_transform, ShapeCollisionCapsule& a_capsule)
		{
			const int longest = a_half[0] >= a_half[1] && a_half[0] >= a_half[2] ? 0 : (a_half[1] >= a_half[2] ? 1 : 2);
			const float radius = 0.5f * (a_half[(longest + 1) % 3] + a_half[(longest + 2) % 3]);
			const float reach = std::max(a_half[longest] - radius, 0.0f);

			const auto endpoint = [&](float a_sign) {
				float local[4] = { a_centre[0], a_centre[1], a_centre[2], 0.0f };
				local[longest] += a_sign * reach;
				return TransformHavokPoint(a_transform, RE::hkVector4{ local[0], local[1], local[2], 0.0f });
			};
			a_capsule.pointA = endpoint(-1.0f);
			a_capsule.pointB = endpoint(1.0f);
			a_capsule.radius = radius * RE::bhkWorld::GetWorldScaleInverse();
			return IsCapsuleFinite(a_capsule);
		}

		/** @brief A world-space capsule along the longest side of a shape's bounds in the body's own frame, or false for a shape this cannot bound. */
		bool BoundShapeCapsule(const RE::hkpShape* a_shape, const RE::hkTransform& a_transform, ShapeCollisionCapsule& a_capsule)
		{
			if (a_shape->type == RE::hkpShapeType::kCapsule)
				return WorldCapsule(*static_cast<const RE::hkpCapsuleShape*>(a_shape), a_transform, a_capsule);
			if (a_shape->type == RE::hkpShapeType::kList || a_shape->type == RE::hkpShapeType::kCompound || a_shape->type == RE::hkpShapeType::kMOPP)
				return false;

			const auto extent = [a_shape](float x, float y, float z) {
				return a_shape->GetMaximumProjection(RE::hkVector4{ x, y, z, 0.0f });
			};
			const float high[3] = { extent(1.0f, 0.0f, 0.0f), extent(0.0f, 1.0f, 0.0f), extent(0.0f, 0.0f, 1.0f) };
			const float low[3] = { -extent(-1.0f, 0.0f, 0.0f), -extent(0.0f, -1.0f, 0.0f), -extent(0.0f, 0.0f, -1.0f) };
			float centre[3];
			float half[3];
			for (int axis = 0; axis < 3; ++axis) {
				centre[axis] = 0.5f * (high[axis] + low[axis]);
				half[axis] = 0.5f * (high[axis] - low[axis]);
			}
			return BoxCapsule(centre, half, a_transform, a_capsule);
		}
	}

	size_t GetRigidBodyCapsules(RE::bhkRigidBody* a_body, std::span<ShapeCollisionCapsule> a_capsules)
	{
		auto* rigid = a_body ? skyrim_cast<RE::hkpRigidBody*>(a_body->referencedObject.get()) : nullptr;
		const auto* shape = rigid && rigid->world ? rigid->collidable.GetShape() : nullptr;
		if (!shape || a_capsules.empty())
			return 0;

		RE::hkTransform transform;
		a_body->GetTransform(transform);

		if (const auto* list = skyrim_cast<const RE::hkpListShape*>(shape)) {
			size_t count = 0;
			const auto children = static_cast<size_t>(list->childInfo.size());
			if (children > 0 && children <= a_capsules.size()) {
				for (size_t index = 0; index < children; ++index) {
					const auto* child = list->childInfo[static_cast<int>(index)].shape;
					if (child && BoundShapeCapsule(child, transform, a_capsules[count]))
						++count;
				}
				if (count > 0)
					return count;
			}

			alignas(16) float centre[4];
			alignas(16) float half[4];
			_mm_store_ps(centre, list->aabbCenter.quad);
			_mm_store_ps(half, list->aabbHalfExtents.quad);
			return BoxCapsule(centre, half, transform, a_capsules[0]) ? 1 : 0;
		}
		return BoundShapeCapsule(shape, transform, a_capsules[0]) ? 1 : 0;
	}

	bool ExtractShapeBound(const RE::hkpShape* shape, float& radius)
	{
		using ShapeType = RE::hkpShapeType;
		if (!shape)
			return false;

		auto project = [shape](float x, float y, float z) {
			return shape->GetMaximumProjection(RE::hkVector4{ x, y, z, 0.0f }) * RE::bhkWorld::GetWorldScaleInverse();
		};
		auto symmetricHalfExtents = [&project](float& hx, float& hy, float& hz) {
			float x_pos = project(1.0f, 0.0f, 0.0f);
			float x_neg = project(-1.0f, 0.0f, 0.0f);
			float y_pos = project(0.0f, 1.0f, 0.0f);
			float y_neg = project(0.0f, -1.0f, 0.0f);
			float z_pos = project(0.0f, 0.0f, 1.0f);
			float z_neg = project(0.0f, 0.0f, -1.0f);
			hx = 0.5f * (x_pos - x_neg);
			hy = 0.5f * (y_pos - y_neg);
			hz = 0.5f * (z_pos - z_neg);
		};
		auto halfDiagonal = [](float hx, float hy, float hz) {
			return sqrtf(hx * hx + hy * hy + hz * hz);
		};
		float hx, hy, hz;
		symmetricHalfExtents(hx, hy, hz);
		if (shape->type == ShapeType::kSphere) {
			radius = hx;
		} else if (shape->type == ShapeType::kBox) {
			radius = halfDiagonal(hx, hy, hz);
		} else if (shape->type == ShapeType::kCylinder) {
			const float hr = std::max(hx, hy);
			radius = sqrtf(hr * hr + hz * hz);
		} else {
			radius = std::max(hx, std::max(hy, hz));
		}
		return true;
	}
}
