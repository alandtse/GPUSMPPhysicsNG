#include "Features/SmpPhysics.h"
#include "smp/CardVertices.h"
#include "smp/RestRetarget.h"

#include "RE/A/Actor.h"
#include "RE/B/BSDynamicTriShape.h"
#include "RE/B/BSGeometry.h"
#include "RE/B/BipedAnim.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiSkinData.h"
#include "RE/N/NiSkinInstance.h"
#include "RE/N/NiSkinPartition.h"
#include "RE/T/TESObjectARMO.h"
#include "Utils/MathUtils.h"
#include "Utils/SkinVertexBlock.h"
#include "Utils/StringUtils.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace
{
	using SmpPhysics::BoneSphere;
	using SmpPhysics::Collider;
	using SmpPhysics::Length;
	using SmpPhysics::RigShape;
	using SmpPhysics::SkinnedVertex;
	using SmpPhysics::Transform;
	using SmpPhysics::Vec3;

	/** @brief A dynamic tri shape stores its morphing vertices as 16 bytes each: a float3 and a pad. */
	constexpr std::uint32_t kDynamicVertexStride = 16;
	/** @brief Vertices a single shape may contribute; a runaway count would otherwise exhaust memory at claim time. */
	constexpr std::uint32_t kMaximumShapeVertices = 200000;
	/** @brief Most skin bones a mesh may name before the reader stops trusting its vertex indices. */
	constexpr std::uint32_t kMaximumSkinBones = 256;
	/** @brief Body vertices one claim may pose before it stops measuring and keeps its default radii. */
	constexpr std::size_t kMaximumBodyVertices = 400000;
	/** @brief Starting value of the hash a body scan carries, so an empty walk still has a stable signature. */
	/** @brief Seconds between two hashes of the meshes an actor is wearing, on the path that notices an armour swap. */
	constexpr double kRefitCheckSeconds = 1.0;
	using SmpPhysics::kHeadBoneName;
	constexpr const char* kNeckName = "NPC Neck [Neck]";
	constexpr const char* kSpine0Name = "NPC Spine [Spn0]";
	constexpr const char* kSpine1Name = "NPC Spine1 [Spn1]";
	constexpr const char* kSpineName = "NPC Spine2 [Spn2]";
	constexpr const char* kPelvisName = "NPC Pelvis [Pelv]";
	constexpr const char* kLeftClavicleName = "NPC L Clavicle [LClv]";
	constexpr const char* kRightClavicleName = "NPC R Clavicle [RClv]";
	constexpr const char* kLeftUpperArmName = "NPC L UpperArm [LUar]";
	constexpr const char* kRightUpperArmName = "NPC R UpperArm [RUar]";
	constexpr const char* kLeftForearmName = "NPC L Forearm [LLar]";
	constexpr const char* kRightForearmName = "NPC R Forearm [RLar]";
	/** @brief Vertex against triangle tests one audit may run before it gives up, so a stray click cannot stall the frame. */
	constexpr std::uint64_t kMaximumAuditTests = 40000000ull;
	/** @brief Height, in Skyrim units, of one bucket of the audit's inside-vertex height histogram. */
	constexpr float kAuditHeightBucket = 10.0f;
	/** @brief Most hair bones the audit lists for the vertices behind a target. */
	constexpr std::size_t kAuditBoneRows = 12;
	/** @brief How close to a surface a vertex must be for an overlap audit to count it as through that surface, in game units. */
	constexpr float kOverlapNearReach = 3.0f;
	constexpr float kAuditPenetrationPercentile = 0.95f;  ///< Fraction of sorted penetrations reported by the audit, unitless.
	constexpr float kDefaultTorsoHeight = 20.0f;          ///< Height of the fallback torso capsule along Z, Skyrim units.

	/** @brief A shape's vertex data, however that shape stores it. */
	struct ShapeSource
	{
		const std::uint8_t* data{};
		std::uint32_t stride{};
		std::uint32_t count{};
		std::uint32_t skinOffset{};
		bool skinned{};
	};

	/** @brief Locates the vertices a_object's first partition renders from. */
	bool ResolveShapeSource(RE::NiAVObject* a_object, ShapeSource& a_source, std::string& a_reason)
	{
		if (!a_object) {
			a_reason = "does not exist";
			return false;
		}
		if (auto* dynamic = a_object->AsDynamicTriShape()) {
			const auto& runtime = dynamic->GetDynamicTrishapeRuntimeData();
			if (!runtime.dynamicData || runtime.dataSize < kDynamicVertexStride) {
				a_reason = "holds no vertices";
				return false;
			}
			a_source.data = static_cast<const std::uint8_t*>(runtime.dynamicData);
			a_source.stride = kDynamicVertexStride;
			a_source.count = runtime.dataSize / kDynamicVertexStride;
			return a_source.count > 0;
		}

		auto* geometry = a_object->AsGeometry();
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		RE::NiSkinPartition* partition = skin ? skin->skinPartition.get() : nullptr;
		if (!partition || partition->partitions.empty()) {
			a_reason = "is not a skinned shape";
			return false;
		}
		const RE::NiSkinPartition::Partition& first = partition->partitions.front();
		RE::BSGraphics::TriShape* buffer = first.buffData;
		if (!buffer || !buffer->rawVertexData) {
			a_reason = "has no vertex buffer";
			return false;
		}
		RE::BSGraphics::VertexDesc desc = first.vertexDesc;
		std::uint32_t stride = desc.GetSize();
		if (stride < sizeof(float) * 4) {
			desc = buffer->vertexDesc;
			stride = desc.GetSize();
		}
		const std::uint32_t vertexCount = partition->vertexCount != 0 ? partition->vertexCount : first.vertices;
		if (stride < sizeof(float) * 4 || vertexCount == 0 || vertexCount > kMaximumShapeVertices) {
			a_reason = "has an unusable vertex layout";
			return false;
		}
		a_source.data = buffer->rawVertexData;
		a_source.stride = stride;
		a_source.count = vertexCount;
		a_source.skinned = desc.HasFlag(RE::BSGraphics::Vertex::VF_SKINNED);
		a_source.skinOffset = desc.GetAttributeOffset(RE::BSGraphics::Vertex::VA_SKINNING);
		if (a_source.skinned && a_source.skinOffset + Util::kSkinVertexBlockBytes > stride) {
			a_reason = "has its skinning block outside its vertex stride";
			return false;
		}
		return true;
	}

	/** @brief Reads a shape's vertex positions, and the four bones each vertex weights when it carries skinning. */
	bool ReadShape(RE::NiAVObject* a_object, bool a_wantSkinning, std::vector<SkinnedVertex>& a_vertices, std::string& a_reason)
	{
		ShapeSource source;
		if (!ResolveShapeSource(a_object, source, a_reason))
			return false;
		a_vertices.clear();
		a_vertices.reserve(source.count);
		for (std::uint32_t index = 0; index < source.count; ++index) {
			const std::uint8_t* vertex = source.data + static_cast<std::size_t>(index) * source.stride;
			const float* position = reinterpret_cast<const float*>(vertex);
			SkinnedVertex read;
			read.position = { position[0], position[1], position[2] };
			if (a_wantSkinning && source.skinned) {
				const Util::SkinVertexBlock block = Util::ReadSkinVertexBlock(vertex + source.skinOffset);
				for (int slot = 0; slot < 4; ++slot) {
					read.bone[slot] = block.paletteSlot[static_cast<std::size_t>(slot)];
					read.weight[slot] = block.weight[static_cast<std::size_t>(slot)];
				}
			}
			a_vertices.push_back(read);
		}
		return !a_vertices.empty();
	}

	bool ReadShapePositions(RE::NiAVObject* a_object, std::vector<Vec3>& a_positions, std::string& a_reason)
	{
		std::vector<SkinnedVertex> vertices;
		if (!ReadShape(a_object, false, vertices, a_reason))
			return false;
		a_positions.clear();
		a_positions.reserve(vertices.size());
		for (const SkinnedVertex& vertex : vertices)
			a_positions.push_back(vertex.position);
		return true;
	}

	/** @brief The triangle shape whose name contains a_hint, preferring one the hair shape's collides-with list names. */
	const RigShape* PickTriangleShape(const SmpPhysics::RigCollision& a_collision, const RigShape& a_hair, const char* a_hint)
	{
		const RigShape* fallback = nullptr;
		for (const RigShape& shape : a_collision.triangleShapes) {
			if (Util::ToLowerAscii(shape.name).find(a_hint) == std::string::npos)
				continue;
			if (!shape.tag.empty() && SmpPhysics::ShapesCanCollide(a_hair, shape))
				return &shape;
			if (fallback == nullptr)
				fallback = &shape;
		}
		return fallback;
	}

	/** @brief The rig's per-vertex shapes, highest priority first, so a rig whose preferred mesh the actor lacks falls through to the next one. */
	std::vector<const RigShape*> OrderHairShapes(const SmpPhysics::RigCollision& a_collision)
	{
		std::vector<const RigShape*> ordered;
		ordered.reserve(a_collision.vertexShapes.size());
		for (const RigShape& shape : a_collision.vertexShapes)
			ordered.push_back(&shape);
		std::ranges::stable_sort(ordered, [](const RigShape* a_left, const RigShape* a_right) {
			return a_left->priority > a_right->priority;
		});
		return ordered;
	}

	/** @brief Whether a mesh is skinned to a bone the rig simulates, which marks a cloth shape among a rig's triangle shapes. */
	bool SkinsToDynamicBone(RE::NiAVObject* a_object, const SmpPhysics::SmpRig& a_rig, const hdt::IMeshClaimContext& a_context)
	{
		auto* geometry = a_object ? a_object->AsGeometry() : nullptr;
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		if (!skin || !skin->bones || !skin->skinData)
			return false;
		const std::uint32_t skinBones = std::min<std::uint32_t>(skin->skinData->GetBoneCount(), kMaximumSkinBones);
		for (std::uint32_t index = 0; index < skinBones; ++index) {
			const RE::NiAVObject* bone = skin->bones[index];
			if (!bone || !bone->name.c_str())
				continue;
			for (const SmpPhysics::RigBone& rigBone : a_rig.bones) {
				const char* merged = a_context.getMergedBoneName(rigBone.name.c_str());
				if (!rigBone.kinematic && (rigBone.name == bone->name.c_str() || (merged && *merged && std::string_view(merged) == bone->name.c_str())))
					return true;
			}
		}
		return false;
	}

	/** @brief The first geometry below a_root that is skinned to a simulated rig bone. */
	RE::NiAVObject* FindClothMesh(RE::NiAVObject* a_root, const SmpPhysics::SmpRig& a_rig, const hdt::IMeshClaimContext& a_context)
	{
		if (!a_root)
			return nullptr;
		if (a_root->AsGeometry())
			return SkinsToDynamicBone(a_root, a_rig, a_context) ? a_root : nullptr;
		if (RE::NiNode* node = a_root->AsNode()) {
			for (auto& child : node->GetChildren()) {
				if (RE::NiAVObject* found = FindClothMesh(child.get(), a_rig, a_context))
					return found;
			}
		}
		return nullptr;
	}

	RE::NiAVObject* FindNode(RE::NiNode* a_root, const hdt::IMeshClaimContext& a_context, const std::string& a_name)
	{
		if (!a_root || a_name.empty())
			return nullptr;
		const char* mergedName = a_context.getMergedBoneName(a_name.c_str());
		return a_root->GetObjectByName(RE::BSFixedString(mergedName && *mergedName ? mergedName : a_name.c_str()));
	}

	/** @brief A shape's vertices with the skinning needed to pose them at the current frame. */
	struct SkinnedShape
	{
		std::vector<Vec3> positions;
		std::vector<std::array<int, 4>> bones;
		std::vector<std::array<float, 4>> weights;
		std::vector<RE::NiAVObject*> skinBones;  ///< not owned: a reference taken on a bone mid-destruction is released into a second delete
		std::vector<Transform> skinToBone;
		std::vector<std::uint16_t> triangles;  ///< three skin bone local indices each
		std::vector<Transform> boneWorld;      ///< this frame's world transform of each skin bone
		bool strips{};
	};

	/** @brief Reads a shape's vertices and the bone transforms that pose them; its triangles are left unread. */
	bool ReadSkinnedVertices(RE::NiAVObject* a_object, SkinnedShape& a_shape, std::string& a_reason)
	{
		std::vector<SkinnedVertex> vertices;
		if (!ReadShape(a_object, true, vertices, a_reason))
			return false;
		auto* geometry = a_object ? a_object->AsGeometry() : nullptr;
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		if (!skin || !skin->bones || !skin->skinData) {
			a_reason = "is not a skinned shape";
			return false;
		}

		const std::uint32_t skinBones = std::min<std::uint32_t>(skin->skinData->GetBoneCount(), kMaximumSkinBones);
		a_shape.skinBones.assign(skinBones, nullptr);
		a_shape.boneWorld.resize(skinBones);
		a_shape.skinToBone.resize(skinBones);
		for (std::uint32_t index = 0; index < skinBones; ++index) {
			RE::NiAVObject* bone = skin->bones[index];
			a_shape.skinBones[index] = bone;
			a_shape.boneWorld[index] = bone != nullptr ? ToSolverTransform(bone->world) : Transform{};
			a_shape.skinToBone[index] = ToSolverTransform(skin->skinData->GetBoneDataSkinToBone(index));
		}

		a_shape.positions.reserve(vertices.size());
		a_shape.bones.reserve(vertices.size());
		a_shape.weights.reserve(vertices.size());
		for (const SkinnedVertex& vertex : vertices) {
			a_shape.positions.push_back(vertex.position);
			std::array<int, 4> bones{ -1, -1, -1, -1 };
			std::array<float, 4> weights{};
			for (int slot = 0; slot < 4; ++slot) {
				const int bone = vertex.bone[slot];
				// A slot naming a bone the instance no longer has cannot pose the vertex, so it is dropped
				// rather than being blended against the identity a missing world transform defaults to.
				if (bone < 0 || bone >= static_cast<int>(skinBones) || skin->bones[bone] == nullptr)
					continue;
				bones[static_cast<std::size_t>(slot)] = bone;
				weights[static_cast<std::size_t>(slot)] = vertex.weight[slot];
			}
			a_shape.bones.push_back(bones);
			a_shape.weights.push_back(weights);
		}
		return true;
	}

	/** @brief Reads the same vertices, plus the triangles they form, for a vertex against triangle test. */
	bool ReadSkinnedShape(RE::NiAVObject* a_object, SkinnedShape& a_shape, std::string& a_reason)
	{
		if (!ReadSkinnedVertices(a_object, a_shape, a_reason))
			return false;
		a_shape.strips = false;
		return SmpPhysics::ReadCardTriangles(a_object, a_shape.triangles, a_reason);
	}

	/** @brief One vertex of a_shape as the plain struct the skinning helpers take. */
	SkinnedVertex VertexOf(const SkinnedShape& a_shape, std::size_t a_vertex)
	{
		SkinnedVertex vertex;
		vertex.position = a_shape.positions[a_vertex];
		for (int slot = 0; slot < 4; ++slot) {
			vertex.bone[slot] = a_shape.bones[a_vertex][static_cast<std::size_t>(slot)];
			vertex.weight[slot] = a_shape.weights[a_vertex][static_cast<std::size_t>(slot)];
		}
		return vertex;
	}

	/** @brief A vertex skinned by the nodes' current world transforms, as the renderer would draw it. */
	Vec3 Posed(const SkinnedShape& a_shape, std::size_t a_vertex)
	{
		return SmpPhysics::SkinVertexToWorld(VertexOf(a_shape, a_vertex), a_shape.boneWorld, a_shape.skinToBone);
	}

	/** @brief A geometry's vertices in world space, skinned to the current pose when the shape carries skinning. */
	bool ReadPosedPositions(RE::NiAVObject* a_object, std::vector<Vec3>& a_positions, std::string& a_reason)
	{
		SkinnedShape shape;
		if (!ReadSkinnedVertices(a_object, shape, a_reason))
			return ReadShapePositions(a_object, a_positions, a_reason);
		a_positions.resize(shape.positions.size());
		for (std::size_t index = 0; index < shape.positions.size(); ++index)
			a_positions[index] = Posed(shape, index);
		return !a_positions.empty();
	}

	/** @brief Whether a_object is a shape a body measurement may read: a geometry the skeleton skins, never a blood decal. */
	bool IsSkinnedGeometry(RE::NiAVObject* a_object)
	{
		auto* geometry = a_object ? a_object->AsGeometry() : nullptr;
		if (geometry && geometry->AsSkinnedDecalTriShape())
			return false;
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		return skin && skin->bones && skin->skinData;
	}

	/** @brief Whether a node carries one of the names a measurement should leave alone. */
	bool IsNamed(const RE::NiAVObject* a_object, const std::vector<std::string>& a_names)
	{
		const char* name = a_object ? a_object->name.c_str() : nullptr;
		if (!name)
			return false;
		for (const std::string& candidate : a_names) {
			if (!candidate.empty() && candidate == name)
				return true;
		}
		return false;
	}

	/**
	 * @brief Biped slots no worn mesh is read from: the hair's own slots, the limbs and held items hair does not
	 *        rest on, and every slot a weapon occupies.
	 */
	constexpr std::array<RE::BIPED_OBJECT, 12> kSkippedArmorSlots = {
		RE::BIPED_OBJECTS::kHead,
		RE::BIPED_OBJECTS::kHair,
		RE::BIPED_OBJECTS::kHands,
		RE::BIPED_OBJECTS::kRing,
		RE::BIPED_OBJECTS::kFeet,
		RE::BIPED_OBJECTS::kCalves,
		RE::BIPED_OBJECTS::kShield,
		RE::BIPED_OBJECTS::kLongHair,
		RE::BIPED_OBJECTS::kCirclet,
		RE::BIPED_OBJECTS::kEars,
		RE::BIPED_OBJECTS::kDecapitateHead,
		RE::BIPED_OBJECTS::kDecapitate
	};

	bool IsSkippedArmorSlot(std::uint32_t a_slot) noexcept
	{
		if (a_slot >= RE::BIPED_OBJECTS::kHandToHandMelee)
			return true;
		const auto slot = static_cast<RE::BIPED_OBJECT>(a_slot);
		return std::ranges::find(kSkippedArmorSlots, slot) != kSkippedArmorSlots.end();
	}

	/** @brief Mixes a pointer into a running signature, so the same worn meshes always hash to the same value. */
	std::uint64_t HashPointer(std::uint64_t a_hash, const void* a_value) noexcept
	{
		return Util::Fnv1a64(std::string_view{ reinterpret_cast<const char*>(&a_value), sizeof(a_value) }, a_hash);
	}

	/** @brief A collider's type as the audit names it. */
	const char* ColliderTypeName(Collider::Type a_type) noexcept
	{
		switch (a_type) {
		case Collider::Type::Ellipsoid:
			return "ellipsoid";
		case Collider::Type::Capsule:
			return "capsule";
		case Collider::Type::Plane:
			return "plane";
		}
		return "unknown";
	}

	/** @brief One vertex of the actor's worn geometry, posed at the frame the scan ran. */
	struct BodySample
	{
		Vec3 world;
		int segmentBone{ -1 };  ///< index into the scan's node list, or -1 for a vertex skinned to anything else
	};

	/** @brief What one body scan needs to know about the claim that is measuring. */
	struct BodyScanInput
	{
		RE::NiAVObject* root{};                                 ///< the top of the scene graph to walk
		std::vector<std::string> hairNames;                     ///< the meshes the rig builds its own shapes from
		std::vector<RE::NiAVObject*> segmentNodes;              ///< every node the body segments name, deduplicated
		std::function<bool(const RE::NiAVObject*)> isHairBone;  ///< whether a skin bone is one of the rig's dynamic bones
		bool measure{};                                         ///< whether to pose the meshes, or only hash them
	};

	/** @brief What one body scan found. */
	struct BodyScanResult
	{
		std::vector<BodySample> samples;  ///< one per body vertex, filled only by a measuring scan
		std::uint64_t signature{};        ///< hashes the worn meshes, so an equipment change is noticed
		std::size_t vertexCount{};        ///< body vertices the scan posed
		bool tooManyVertices{};           ///< the walk passed kMaximumBodyVertices, so nothing can be measured
	};

	/**
	 * @brief Walks the actor's worn skinned geometry, hashing it and optionally posing what the segments measure.
	 * @details A mesh named by the rig's own shapes is the hair and is never hashed or measured, so the cheap
	 *          signature pass and a full measurement always agree on which meshes the actor is wearing.
	 */
	void ScanBody(const BodyScanInput& a_input, BodyScanResult& a_result)
	{
		a_result = {};
		std::uint64_t signature = Util::kFnv1a64Basis;
		bool measuring = a_input.measure;

		std::vector<RE::NiPointer<RE::NiAVObject>> pending{ RE::NiPointer<RE::NiAVObject>(a_input.root) };
		while (!pending.empty()) {
			const RE::NiPointer<RE::NiAVObject> held = std::move(pending.back());
			pending.pop_back();
			RE::NiAVObject* object = held.get();
			if (!object)
				continue;
			if (RE::NiNode* node = object->AsNode()) {
				for (auto& child : node->GetChildren())
					pending.emplace_back(child.get());
			}
			if (!IsSkinnedGeometry(object) || IsNamed(object, a_input.hairNames))
				continue;

			auto* geometry = object->AsGeometry();
			RE::NiSkinInstance* skin = geometry->GetGeometryRuntimeData().skinInstance.get();
			signature = HashPointer(signature, object);
			signature = HashPointer(signature, skin);
			if (!measuring)
				continue;

			SkinnedShape shape;
			std::string reason;
			if (!ReadSkinnedVertices(object, shape, reason) || shape.skinBones.empty())
				continue;

			std::vector<int> segmentOfSkinBone(shape.skinBones.size(), -1);
			std::vector<bool> hairSkinBone(shape.skinBones.size(), false);
			for (std::size_t bone = 0; bone < shape.skinBones.size(); ++bone) {
				const RE::NiAVObject* node = shape.skinBones[bone];
				hairSkinBone[bone] = a_input.isHairBone(node);
				for (std::size_t segment = 0; segment < a_input.segmentNodes.size(); ++segment) {
					const RE::NiAVObject* segmentNode = a_input.segmentNodes[segment];
					if (segmentNode == nullptr || node == nullptr)
						continue;
					if (segmentNode == node || segmentNode->name == node->name) {
						segmentOfSkinBone[bone] = static_cast<int>(segment);
						break;
					}
				}
			}

			const std::size_t first = a_result.samples.size();
			std::size_t onHairBones = 0;
			for (std::size_t index = 0; index < shape.positions.size(); ++index) {
				const SkinnedVertex vertex = VertexOf(shape, index);
				// A vertex no valid bone weights is left in mesh space by the blend, so it would be measured
				// wherever the mesh happens to sit rather than where the renderer draws it.
				float poseWeight = 0.0f;
				for (int slot = 0; slot < 4; ++slot) {
					if (vertex.bone[slot] >= 0)
						poseWeight += vertex.weight[slot];
				}
				if (!(poseWeight > 0.0f))
					continue;

				const int bone = SmpPhysics::DominantBone(vertex, static_cast<int>(shape.skinBones.size()));
				int segmentBone = -1;
				if (bone >= 0) {
					const auto slot = static_cast<std::size_t>(bone);
					if (hairSkinBone[slot])
						++onHairBones;
					segmentBone = segmentOfSkinBone[slot];
				}
				a_result.samples.push_back({ SmpPhysics::SkinVertexToWorld(vertex, shape.boneWorld, shape.skinToBone), segmentBone });
			}
			if (!shape.positions.empty() && onHairBones * 2 > shape.positions.size()) {
				a_result.samples.resize(first);
				continue;
			}
			a_result.vertexCount += shape.positions.size();
			if (a_result.vertexCount > kMaximumBodyVertices) {
				a_result.tooManyVertices = true;
				a_result.samples.clear();
				measuring = false;
			}
		}

		a_result.signature = signature;
	}

	/** @brief Closest point on a triangle to a point, and the squared distance to it. */
	float ClosestOnTriangle(const Vec3& a_point, const Vec3& a_first, const Vec3& a_second, const Vec3& a_third, Vec3& a_closest)
	{
		const Vec3 ab = a_second - a_first;
		const Vec3 ac = a_third - a_first;
		const Vec3 ap = a_point - a_first;
		const float d1 = Dot(ab, ap);
		const float d2 = Dot(ac, ap);
		const auto finish = [&](const Vec3& a_nearest) {
			a_closest = a_nearest;
			const Vec3 difference = a_point - a_nearest;
			return Dot(difference, difference);
		};
		if (d1 <= 0.0f && d2 <= 0.0f)
			return finish(a_first);

		const Vec3 bp = a_point - a_second;
		const float d3 = Dot(ab, bp);
		const float d4 = Dot(ac, bp);
		if (d3 >= 0.0f && d4 <= d3)
			return finish(a_second);
		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
			return finish(a_first + ab * (d1 / (d1 - d3)));

		const Vec3 cp = a_point - a_third;
		const float d5 = Dot(ab, cp);
		const float d6 = Dot(ac, cp);
		if (d6 >= 0.0f && d5 <= d6)
			return finish(a_third);
		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
			return finish(a_first + ac * (d2 / (d2 - d6)));
		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
			return finish(a_second + (a_third - a_second) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));

		const float inverse = 1.0f / (va + vb + vc);
		return finish(a_first + ab * (vb * inverse) + ac * (vc * inverse));
	}
	/** @brief The top of the scene graph, which is as far up as GetObjectByName can search down from. */
	RE::NiAVObject* FindMeshRoot(const hdt::IMeshClaimContext& a_context)
	{
		if (RE::Actor* actor = a_context.getActor()) {
			if (RE::NiAVObject* root = actor->Get3D())
				return root;
		}
		RE::NiAVObject* top = a_context.getSkeleton();
		while (top && top->parent)
			top = top->parent;
		return top;
	}

	/** @brief A mesh by name, looked up in the claimed model first because it is not attached to the actor yet when SMP offers it. */
	RE::NiAVObject* FindMeshObject(const hdt::IMeshClaimContext& a_context, RE::NiAVObject* a_actorRoot, const std::string& a_name)
	{
		if (a_name.empty())
			return nullptr;
		const RE::BSFixedString name(a_name.c_str());
		if (RE::NiAVObject* model = a_context.getModel()) {
			if (model->name == name)
				return model;
			if (RE::NiNode* node = model->AsNode()) {
				if (RE::NiAVObject* found = node->GetObjectByName(name))
					return found;
			}
		}
		return a_actorRoot ? a_actorRoot->GetObjectByName(name) : nullptr;
	}

	/** @brief The world point the hair hangs from: the first kinematic rig bone, or the first driver when the rig names none. */
	bool FindHairAnchor(const SmpPhysics::SmpRig& a_rig, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes,
		const std::vector<RE::NiPointer<RE::NiAVObject>>& a_drivers, Vec3& a_anchor)
	{
		for (std::size_t index = 0; index < a_rig.bones.size() && index < a_boneNodes.size(); ++index) {
			if (!a_rig.bones[index].kinematic || !a_boneNodes[index])
				continue;
			a_anchor = ToSolverTransform(a_boneNodes[index]->world).translation;
			return true;
		}
		for (const RE::NiPointer<RE::NiAVObject>& driver : a_drivers) {
			if (!driver)
				continue;
			a_anchor = ToSolverTransform(driver->world).translation;
			return true;
		}
		return false;
	}
}

json SmpPhysicsFeature::DescribeClaimColliders(const Claim& a_claim)
{
	json described = json::array();
	for (std::size_t index = 0; index < a_claim.colliders.size(); ++index) {
		const SmpPhysics::Collider& collider = a_claim.colliders[index];
		const ClaimCollision::ColliderFit fit = index < a_claim.colliderFits.size() ? a_claim.colliderFits[index] : ClaimCollision::ColliderFit{};
		const char* driverName = "";
		if (collider.driverIndex >= 0 && static_cast<std::size_t>(collider.driverIndex) < a_claim.collisionDrivers.size()) {
			if (const RE::NiAVObject* node = a_claim.collisionDrivers[static_cast<std::size_t>(collider.driverIndex)].get()) {
				if (const char* name = node->name.c_str())
					driverName = name;
			}
		}
		described.push_back({ { "type", ColliderTypeName(collider.type) },
			{ "driver", driverName },
			{ "radius", collider.radius },
			{ "fitted", fit.fitted },
			{ "vertexCount", fit.vertexCount } });
	}
	for (const SmpPhysics::SkinnedSurface& surface : a_claim.surfaces) {
		int driven = 0;
		for (const int driver : surface.boneDriver)
			driven += driver >= 0 ? 1 : 0;
		const std::vector<SmpPhysics::Vec3> posed = SmpPhysics::PoseSurface(surface, a_claim.driverWorld);
		SmpPhysics::Vec3 low{ FLT_MAX, FLT_MAX, FLT_MAX };
		SmpPhysics::Vec3 high{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
		for (const SmpPhysics::Vec3& point : posed) {
			low = { std::min(low.x, point.x), std::min(low.y, point.y), std::min(low.z, point.z) };
			high = { std::max(high.x, point.x), std::max(high.y, point.y), std::max(high.z, point.z) };
		}
		const SmpPhysics::Vec3 head = a_claim.driverWorld.empty() ? SmpPhysics::Vec3{} : a_claim.driverWorld.front().translation;
		described.push_back({ { "type", "surface" },
			{ "bounds", { low.x, low.y, low.z, high.x, high.y, high.z } },
			{ "firstDriver", { head.x, head.y, head.z } },
			{ "name", surface.name },
			{ "vertexCount", surface.vertices.size() },
			{ "triangles", surface.indices.size() / 3 },
			{ "skinBones", surface.boneDriver.size() },
			{ "drivenSkinBones", driven },
			{ "margin", surface.margin },
			{ "contactPenetration", surface.contactPenetration },
			{ "penetration", surface.penetration } });
	}
	return described;
}

json SmpPhysicsFeature::DescribeAuditClaims(RE::FormID a_formID) const
{
	const std::scoped_lock lock(stateMutex);
	json described = json::array();
	for (const auto& entry : claims) {
		const Claim& claim = entry.second;
		if (claim.actorFormID != a_formID)
			continue;
		// The bones are placed where they are drawn, whichever backend steps the claim.
		std::vector<SmpPhysics::Transform> boneWorld(claim.nodes.size());
		for (std::size_t bone = 0; bone < boneWorld.size(); ++bone)
			boneWorld[bone] = claim.BoneWorld(bone);
		// The proxies the solver was given: the claim's own, then the slots other bodies fill each frame.
		std::vector<SmpPhysics::Collider> probeColliders = claim.colliders;
		probeColliders.resize(probeColliders.size() + SmpPhysics::kExternalColliderSlots);
		for (std::size_t slot = 0; slot < claim.externalColliders.size() && slot < SmpPhysics::kExternalColliderSlots; ++slot)
			probeColliders[claim.colliders.size() + slot] = claim.externalColliders[slot];
		std::vector<SmpPhysics::SphereProbe> probes;
		SmpPhysics::ProbeSpheres(claim.spheres, probeColliders, claim.surfaces, boneWorld, claim.driverWorld, probes);
		json behindBySphere = json::array();
		for (const auto& probe : probes)
			behindBySphere.push_back(std::round(probe.behind * 100.0f) / 100.0f);
		std::sort(probes.begin(), probes.end(), [](const auto& a_left, const auto& a_right) { return a_left.depth > a_right.depth; });
		json deepest = json::array();
		int inside = 0;
		for (const auto& probe : probes) {
			if (probe.depth > 0.0f)
				++inside;
			if (deepest.size() < 12 && probe.depth > 0.0f)
				deepest.push_back({ { "bone", claim.parsed->rig.bones[static_cast<std::size_t>(probe.bone)].name },
					{ "depth", probe.depth }, { "collider", probe.collider },
					{ "center", { probe.center.x, probe.center.y, probe.center.z } } });
		}
		float farthest = 0.0f;
		double meanOffset = 0.0;
		int behindFace = 0;
		int behindOutward = 0;
		float deepestBehind = 0.0f;
		std::array<int, 6> behindBuckets{};
		int insideShell = 0;
		std::map<std::string, int> behindBones;
		std::map<std::string, int> insideShellBones;
		std::map<std::string, std::array<std::array<int, 6>, 2>> behindSurfaces;
		for (const auto& probe : probes) {
			if (probe.insideShell > 0.0f) {
				++insideShell;
				++insideShellBones[claim.parsed->rig.bones[static_cast<std::size_t>(probe.bone)].name];
			}
			if (probe.behind > SmpPhysics::kProbeLostBehind) {
				++behindFace;
				behindOutward += probe.behindOutward ? 1 : 0;
				deepestBehind = std::max(deepestBehind, probe.behind);
				const std::size_t bucket = probe.behind <= 1.0f ? 0 : probe.behind <= 2.0f ? 1 :
				                                                  probe.behind <= 4.0f     ? 2 :
				                                                  probe.behind <= 8.0f     ? 3 :
				                                                  probe.behind <= 16.0f    ? 4 :
				                                                                             5;
				++behindBuckets[bucket];
				++behindBones[claim.parsed->rig.bones[static_cast<std::size_t>(probe.bone)].name];
				const std::string surfaceName = probe.behindSurface >= 0 && static_cast<std::size_t>(probe.behindSurface) < claim.surfaces.size() ? claim.surfaces[static_cast<std::size_t>(probe.behindSurface)].name : "?";
				++behindSurfaces[surfaceName][probe.behindOutward ? 0 : 1][bucket];
			}
			const float offset = SmpPhysics::Length(probe.center - boneWorld[static_cast<std::size_t>(probe.bone)].translation);
			farthest = std::max(farthest, offset);
			meanOffset += offset;
		}
		described.push_back({ { "token", entry.first },
			{ "sphereOffsetFromBoneMax", farthest },
			{ "sphereOffsetFromBoneMean", probes.empty() ? 0.0 : meanOffset / static_cast<double>(probes.size()) },
			{ "spheresPenetrating", inside },
			{ "retargetBehindSpheres", claim.retargetBehind },
			{ "retargetResidualSpheres", claim.retargetResidual },
			{ "retargetedJoints", claim.retargetedJoints },
			{ "retargetKeptAuthored", claim.retargetKeptAuthored },
			{ "retargetMaxAngle", claim.retargetMaxAngle },
			{ "retargetMaxBoneMove", claim.retargetMaxBoneMove },
			{ "retargetSagMaxMove", claim.retargetSagMaxMove },
			{ "spheresInsideShell", insideShell },
			{ "insideShellBones", insideShellBones },
			{ "spheresBehindFace", behindFace },
			{ "spheresBehindOutwardFace", behindOutward },
			{ "behindFaceDepthBuckets", behindBuckets },
			{ "behindFaceBones", behindBones },
			{ "behindFaceSurfaces", behindSurfaces },
			{ "deepestBehindFace", deepestBehind },
			{ "behindBySphere", std::move(behindBySphere) },
			{ "deepestSpheres", std::move(deepest) },
			{ "file", claim.physicsFile },
			{ "sphereCount", claim.sphereCount },
			{ "exemptSphereCount", claim.exemptSphereCount },
			{ "colliderCount", claim.colliderCount },
			{ "colliders", DescribeClaimColliders(claim) } });
	}
	return described;
}

namespace
{
	/** @brief Measures how far a_hairName's vertices sit behind a_targetName's surface, both posed as drawn, and adds the figures to a_result.
	 *  @param a_boneWorld A world transform to pose a skin bone with instead of its node's, for bones the GPU solver moves.
	 *  @param a_nearReach When above zero, only a vertex this close to the target's nearest triangle counts as penetrating, so two layers of one garment lying far apart are not read as one passing through the other. */
	json MeasureMeshPenetration(RE::NiAVObject* a_root, const std::string& a_hairName, const std::string& a_targetName, json a_result,
		const std::map<const RE::NiAVObject*, Transform>& a_boneWorld = {}, float a_nearReach = 0.0f)
	{
		RE::NiAVObject* hairObject = a_root->GetObjectByName(RE::BSFixedString(a_hairName.c_str()));
		RE::NiAVObject* targetObject = a_root->GetObjectByName(RE::BSFixedString(a_targetName.c_str()));
		if (!hairObject || !targetObject)
			return { { "error", "one of the meshes is not below the actor" } };

		SkinnedShape hair;
		SkinnedShape target;
		std::string reason;
		if (!ReadSkinnedShape(hairObject, hair, reason))
			return { { "error", std::format("the hair mesh '{}' {}", a_hairName, reason) } };
		if (!ReadSkinnedShape(targetObject, target, reason))
			return { { "error", std::format("the target mesh '{}' {}", a_targetName, reason) } };
		if (hair.triangles.empty() && target.triangles.empty())
			return { { "error", "neither mesh has triangles to compare" } };
		for (SkinnedShape* shape : { &hair, &target }) {
			for (std::size_t bone = 0; bone < shape->skinBones.size() && bone < shape->boneWorld.size(); ++bone) {
				if (const auto found = a_boneWorld.find(shape->skinBones[bone]); found != a_boneWorld.end())
					shape->boneWorld[bone] = found->second;
			}
		}

		std::vector<Vec3> hairPoints(hair.positions.size());
		for (std::size_t index = 0; index < hairPoints.size(); ++index)
			hairPoints[index] = Posed(hair, index);
		std::vector<Vec3> targetPoints(target.positions.size());
		for (std::size_t index = 0; index < targetPoints.size(); ++index)
			targetPoints[index] = Posed(target, index);

		const std::size_t triangleCount = target.triangles.size() / 3;
		if (triangleCount == 0)
			return { { "error", std::format("the target mesh '{}' has no triangles", a_targetName) } };
		if (static_cast<std::uint64_t>(hairPoints.size()) * triangleCount > kMaximumAuditTests)
			return { { "error", std::format("{} vertices against {} triangles is more than the {} tests one audit runs",
									hairPoints.size(), triangleCount, kMaximumAuditTests) } };

		Vec3 centre{};
		for (const Vec3& point : targetPoints)
			centre = centre + point;
		centre = centre / static_cast<float>(targetPoints.size());
		float outwardness = 0.0f;
		for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
			const std::uint16_t* indices = target.triangles.data() + triangle * 3;
			if (indices[0] >= targetPoints.size() || indices[1] >= targetPoints.size() || indices[2] >= targetPoints.size())
				continue;
			const Vec3& first = targetPoints[indices[0]];
			const Vec3 normal = SmpPhysics::Cross(targetPoints[indices[1]] - first, targetPoints[indices[2]] - first);
			const Vec3 middle = (first + targetPoints[indices[1]] + targetPoints[indices[2]]) / 3.0f;
			outwardness += SmpPhysics::Dot(normal, middle - centre);
		}
		const float orientation = outwardness < 0.0f ? -1.0f : 1.0f;

		constexpr float kDistanceBinEdges[] = { -1.0f, -0.4f, 0.0f, 0.4f, 0.81f, 1.5f, 3.0f };
		std::size_t distanceHistogram[std::size(kDistanceBinEdges) + 1]{};
		std::vector<float> penetrations;
		float deepest = 0.0f;
		Vec3 deepestAt{};
		const RE::NiAVObject* headNode = a_root->GetObjectByName(RE::BSFixedString(SmpPhysics::kHeadBoneName));
		const Transform headInverse = headNode != nullptr ? ToSolverTransform(headNode->world).Inverse() : Transform{};
		std::map<std::string, std::pair<int, float>> insideByBone;
		std::map<int, int> insideByHeight;
		Vec3 insideLow{ FLT_MAX, FLT_MAX, FLT_MAX };
		Vec3 insideHigh{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
		for (std::size_t pointIndex = 0; pointIndex < hairPoints.size(); ++pointIndex) {
			const Vec3& point = hairPoints[pointIndex];
			float nearest = -1.0f;
			std::size_t nearestTriangle = 0;
			Vec3 closest{};
			for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
				const std::uint16_t* indices = target.triangles.data() + triangle * 3;
				Vec3 candidate;
				bool usable = true;
				for (int corner = 0; corner < 3; ++corner) {
					if (indices[corner] >= targetPoints.size()) {
						usable = false;
						break;
					}
				}
				if (!usable)
					continue;
				const float squared = ClosestOnTriangle(point, targetPoints[indices[0]], targetPoints[indices[1]],
					targetPoints[indices[2]], candidate);
				if (nearest < 0.0f || squared < nearest) {
					nearest = squared;
					nearestTriangle = triangle;
					closest = candidate;
				}
			}
			if (nearest < 0.0f)
				continue;
			const std::uint16_t* indices = target.triangles.data() + nearestTriangle * 3;
			const Vec3 first = targetPoints[indices[0]];
			const Vec3 normal = SmpPhysics::Normalized(
				SmpPhysics::Cross(targetPoints[indices[1]] - first, targetPoints[indices[2]] - first));
			const float signedDistance = orientation * SmpPhysics::Dot(point - closest, normal);
			std::size_t bin = 0;
			while (bin < std::size(kDistanceBinEdges) && signedDistance >= kDistanceBinEdges[bin])
				++bin;
			++distanceHistogram[bin];
			if (signedDistance >= 0.0f || (a_nearReach > 0.0f && nearest > a_nearReach * a_nearReach))
				continue;
			const float depth = -signedDistance;
			penetrations.push_back(depth);
			const Vec3 local = headInverse.TransformPoint(point);
			insideLow = { std::min(insideLow.x, local.x), std::min(insideLow.y, local.y), std::min(insideLow.z, local.z) };
			insideHigh = { std::max(insideHigh.x, local.x), std::max(insideHigh.y, local.y), std::max(insideHigh.z, local.z) };
			++insideByHeight[static_cast<int>(std::floor(local.z / kAuditHeightBucket))];
			const int skinBone = SmpPhysics::DominantBone(VertexOf(hair, pointIndex), static_cast<int>(hair.skinBones.size()));
			const RE::NiAVObject* boneNode = skinBone >= 0 ? hair.skinBones[static_cast<std::size_t>(skinBone)] : nullptr;
			auto& byBone = insideByBone[boneNode != nullptr && boneNode->name.c_str() ? boneNode->name.c_str() : "(none)"];
			++byBone.first;
			byBone.second += depth;
			if (depth > deepest) {
				deepest = depth;
				deepestAt = point;
			}
		}

		a_result["distanceBinEdges"] = std::vector<float>(std::begin(kDistanceBinEdges), std::end(kDistanceBinEdges));
		a_result["distanceHistogram"] = std::vector<std::size_t>(std::begin(distanceHistogram), std::end(distanceHistogram));
		a_result["hairMesh"] = a_hairName;
		a_result["targetMesh"] = a_targetName;
		a_result["windingFlipped"] = orientation < 0.0f;
		a_result["hairVertexCount"] = hairPoints.size();
		a_result["targetTriangleCount"] = triangleCount;
		a_result["penetratingVertexCount"] = penetrations.size();
		a_result["maxPenetration"] = deepest;
		if (!penetrations.empty()) {
			std::sort(penetrations.begin(), penetrations.end());
			float total = 0.0f;
			for (const float depth : penetrations)
				total += depth;
			const std::size_t percentile = std::min(penetrations.size() - 1,
				static_cast<std::size_t>(kAuditPenetrationPercentile * static_cast<float>(penetrations.size() - 1)));
			a_result["percentile95Penetration"] = penetrations[percentile];
			a_result["meanPenetration"] = total / static_cast<float>(penetrations.size());
		} else {
			a_result["percentile95Penetration"] = 0.0f;
			a_result["meanPenetration"] = 0.0f;
		}
		a_result["deepestAt"] = { deepestAt.x, deepestAt.y, deepestAt.z };
		if (!penetrations.empty()) {
			a_result["insideBounds"] = { { "low", { insideLow.x, insideLow.y, insideLow.z } }, { "high", { insideHigh.x, insideHigh.y, insideHigh.z } } };
			json byHeight = json::object();
			for (const auto& [bucket, count] : insideByHeight)
				byHeight[std::format("{}..{}", bucket * static_cast<int>(kAuditHeightBucket), (bucket + 1) * static_cast<int>(kAuditHeightBucket))] = count;
			a_result["insideByHeight"] = std::move(byHeight);
			std::vector<std::pair<std::string, std::pair<int, float>>> ranked(insideByBone.begin(), insideByBone.end());
			std::sort(ranked.begin(), ranked.end(), [](const auto& a_left, const auto& a_right) { return a_left.second.first > a_right.second.first; });
			json byBone = json::array();
			for (std::size_t index = 0; index < ranked.size() && index < kAuditBoneRows; ++index)
				byBone.push_back({ { "bone", ranked[index].first }, { "count", ranked[index].second.first },
					{ "meanDepth", ranked[index].second.second / static_cast<float>(ranked[index].second.first) } });
			a_result["insideByBone"] = std::move(byBone);
		}
		return a_result;
	}
}

json SmpPhysicsFeature::CollisionAuditJson(const json& a_args) const
{
	const RE::FormID formID = a_args.value("actorFormID", 0u);
	const std::string hairName = a_args.value("hairMesh", std::string{});
	const std::string targetName = a_args.value("targetMesh", std::string{});
	if (formID == 0)
		return { { "error", "actorFormID is required" } };
	if (hairName.empty() != targetName.empty())
		return { { "error", "hairMesh and targetMesh are only measured as a pair, so give both or neither" } };

	auto* actor = RE::TESForm::LookupByID<RE::Actor>(formID);
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root)
		return { { "error", "the actor is not loaded" } };

	// The GPU solver moves its bones without writing the engine's nodes, so pose them from its result unless asked for the engine's.
	std::map<const RE::NiAVObject*, Transform> solverWorld;
	std::vector<std::pair<std::string, std::string>> overlapPairs;
	const bool engineOnly = a_args.value("pose", std::string("solver")) == "engine";
	{
		const std::scoped_lock lock(stateMutex);
		for (const auto& entry : claims) {
			const Claim& claim = entry.second;
			if (claim.actorFormID != formID || !claim.parsed)
				continue;
			const SmpPhysics::SmpRig& rig = claim.parsed->rig;
			if (!engineOnly) {
				for (std::size_t bone = 0; bone < rig.bones.size() && bone < claim.nodes.size(); ++bone) {
					if (!rig.bones[bone].kinematic && claim.nodes[bone])
						solverWorld[claim.nodes[bone].get()] = claim.BoneWorld(bone);
				}
			}
			const std::vector<RigShape>& triangleShapes = rig.collision.triangleShapes;
			for (const RigShape& first : triangleShapes) {
				for (const RigShape& second : triangleShapes) {
					if (&first != &second && SmpPhysics::ShapesCanCollide(first, second))
						overlapPairs.emplace_back(first.name, second.name);
				}
			}
		}
	}
	json result = json::object();
	result["claims"] = DescribeAuditClaims(formID);
	result["pose"] = engineOnly ? "engine" : "solver";
	if (hairName.empty()) {
		if (!a_args.value("overlaps", false))
			return result;
		// Every pair of the rig's own shapes whose tags let them collide, measured as drawn: what the eye sees as one piece through another.
		json rows = json::array();
		for (const auto& [first, second] : overlapPairs) {
			json measured = MeasureMeshPenetration(root, first, second, json::object(), solverWorld, kOverlapNearReach);
			json row = { { "hair", first }, { "target", second } };
			for (const char* key : { "error", "hairVertexCount", "penetratingVertexCount", "maxPenetration", "meanPenetration", "percentile95Penetration", "deepestAt", "insideByBone" }) {
				if (measured.contains(key))
					row[key] = measured[key];
			}
			rows.push_back(std::move(row));
		}
		result["overlaps"] = std::move(rows);
		return result;
	}
	return MeasureMeshPenetration(root, hairName, targetName, std::move(result), solverWorld);
}

/** @brief What the pieces of BuildClaimCollision hand to each other. */
struct SmpPhysicsFeature::CollisionBuildState
{
	const hdt::IMeshClaimContext& context;
	const SmpPhysics::SmpRig& rig;
	const std::vector<RE::NiPointer<RE::NiAVObject>>& boneNodes;
	ClaimCollision& collision;
	const SmpPhysics::RigCollision* shapes{};
	const RigShape* hairShape{};
	RE::NiNode* skeleton{};
	RE::NiAVObject* meshRoot{};
	RE::NiAVObject* hairMesh{};
	RE::NiAVObject* headNode{};
	RE::NiAVObject* neckNode{};
	RE::NiAVObject* spineNode{};
	RE::NiAVObject* leftClavicleNode{};
	RE::NiAVObject* rightClavicleNode{};
	RE::NiAVObject* leftUpperArmNode{};
	RE::NiAVObject* rightUpperArmNode{};
	RE::NiAVObject* leftForearmNode{};
	RE::NiAVObject* rightForearmNode{};
	RE::NiAVObject* lowerTorsoNode{};
	RE::NiAVObject* middleSpineNode{};
	const std::vector<SmpPhysics::Transform>* restPose{};  ///< the rest pose the retarget settled on, or null for the nodes' own
	int headDriver{ -1 };
	int neckDriver{ -1 };
	int spineDriver{ -1 };
	int leftClavicleDriver{ -1 };
	int rightClavicleDriver{ -1 };
	int leftArmDriver{ -1 };
	int rightArmDriver{ -1 };
	int lowerTorsoDriver{ -1 };
	int groundDriver{ -1 };
	std::map<std::string, int> rigBoneOfName;
	std::vector<bool> dynamicBone;
	std::vector<std::string> hairNames;                 ///< The meshes the rig builds its own shapes from, which are never body.
	std::map<const RE::NiAVObject*, int> driverOfNode;  ///< Collision driver index of every node already used as one.
	std::vector<RE::NiAVObject*> rigSurfaceMeshes;      ///< The meshes the rig's own collision surfaces were built from.
	std::vector<const RigShape*> sphereShapes;          ///< The shapes the collision spheres were built from; a cloth rig builds them from every triangle shape that skins to its bones.
	std::vector<const RigShape*> boneShape;             ///< Per rig bone, the shape its spheres came from; empty unless sphereShapes holds more than one shape.
	std::vector<RigShape> rigSurfaceShapes;             ///< The shapes the first collision surfaces were built from, in surface order.

	/** @brief A shape is posed through the skeleton nodes its mesh is skinned to, so each becomes a driver shared by every surface built from it. */
	void BindSurfaceDrivers(const SkinnedShape& a_skinned, SmpPhysics::SkinnedSurface& a_surface)
	{
		a_surface.skinToBone = a_skinned.skinToBone;
		a_surface.boneDriver.assign(a_skinned.skinBones.size(), -1);
		for (std::size_t bone = 0; bone < a_skinned.skinBones.size(); ++bone) {
			RE::NiAVObject* node = a_skinned.skinBones[bone];
			if (!node)
				continue;
			const auto known = driverOfNode.find(node);
			if (known != driverOfNode.end()) {
				a_surface.boneDriver[bone] = known->second;
			} else {
				collision.drivers.emplace_back(node);
				a_surface.boneDriver[bone] = static_cast<int>(collision.drivers.size()) - 1;
				driverOfNode.emplace(node, a_surface.boneDriver[bone]);
			}
		}
	}
};

bool SmpPhysicsFeature::ResolveCollisionInputs(CollisionBuildState& s, std::string& a_reason) const
{
	const hdt::IMeshClaimContext& a_context = s.context;
	const SmpPhysics::SmpRig& a_rig = s.rig;
	const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes = s.boneNodes;
	ClaimCollision& a_collision = s.collision;
	const SmpPhysics::RigCollision& shapes = a_rig.collision;
	std::vector<const RigShape*> hairShapeCandidates = OrderHairShapes(shapes);
	if (hairShapeCandidates.empty() && !shapes.triangleShapes.empty())
		hairShapeCandidates.push_back(&shapes.triangleShapes.back());
	if (hairShapeCandidates.empty()) {
		a_reason = "the rig declares no collision shape to build spheres from";
		a_collision.bodyShapes = shapes.triangleShapes;
		return false;
	}

	RE::NiNode* skeleton = a_context.getSkeleton();
	RE::NiAVObject* meshRoot = FindMeshRoot(a_context);
	if (!meshRoot) {
		a_reason = "the actor has no scene graph to look the hair mesh up in";
		return false;
	}
	const RigShape* hairShape = nullptr;
	RE::NiAVObject* hairMesh = nullptr;
	std::string triedShapes;
	const bool clothFallback = shapes.vertexShapes.empty();
	for (const RigShape* candidate : hairShapeCandidates) {
		hairMesh = FindMeshObject(a_context, meshRoot, candidate->name);
		if (clothFallback && !SkinsToDynamicBone(hairMesh, a_rig, a_context))
			hairMesh = FindClothMesh(a_context.getModel(), a_rig, a_context);
		if (hairMesh) {
			hairShape = candidate;
			break;
		}
		if (!triedShapes.empty())
			triedShapes += ", ";
		triedShapes += std::format("'{}'", candidate->name);
	}
	if (hairShape == nullptr) {
		a_reason = std::format("none of the hair meshes {} is below the actor", triedShapes);
		if (shapes.vertexShapes.empty())
			a_collision.bodyShapes = shapes.triangleShapes;
		return false;
	}

	// The drivers are the nodes the colliders are expressed in. Their order is fixed here, because every collider
	// stores the index of the one it follows and the claim rebuilds their world transforms every frame.
	const auto addDriver = [&a_collision](RE::NiAVObject* a_node) {
		if (a_node == nullptr)
			return -1;
		a_collision.drivers.emplace_back(a_node);
		return static_cast<int>(a_collision.drivers.size()) - 1;
	};

	RE::NiAVObject* headNode = nullptr;
	for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
		if (a_rig.bones[index].name == kHeadBoneName) {
			headNode = a_boneNodes[index].get();
			break;
		}
	}
	if (!headNode)
		headNode = FindNode(skeleton, a_context, kHeadBoneName);
	if (!headNode)
		headNode = skeleton;
	RE::NiAVObject* neckNode = FindNode(skeleton, a_context, kNeckName);
	RE::NiAVObject* spineNode = FindNode(skeleton, a_context, kSpineName);
	RE::NiAVObject* leftClavicleNode = FindNode(skeleton, a_context, kLeftClavicleName);
	RE::NiAVObject* rightClavicleNode = FindNode(skeleton, a_context, kRightClavicleName);
	RE::NiAVObject* leftUpperArmNode = FindNode(skeleton, a_context, kLeftUpperArmName);
	RE::NiAVObject* rightUpperArmNode = FindNode(skeleton, a_context, kRightUpperArmName);
	RE::NiAVObject* leftForearmNode = FindNode(skeleton, a_context, kLeftForearmName);
	RE::NiAVObject* rightForearmNode = FindNode(skeleton, a_context, kRightForearmName);
	RE::NiAVObject* lowerTorsoNode = FindNode(skeleton, a_context, kPelvisName);
	if (!lowerTorsoNode)
		lowerTorsoNode = FindNode(skeleton, a_context, kSpine0Name);
	RE::NiAVObject* middleSpineNode = FindNode(skeleton, a_context, kSpine1Name);

	const int headDriver = addDriver(headNode);
	const int neckDriver = addDriver(neckNode);
	const int spineDriver = addDriver(spineNode);
	const int leftClavicleDriver = addDriver(leftClavicleNode);
	const int rightClavicleDriver = addDriver(rightClavicleNode);
	const int leftArmDriver = addDriver(leftUpperArmNode);
	const int rightArmDriver = addDriver(rightUpperArmNode);
	const int lowerTorsoDriver = addDriver(lowerTorsoNode);

	RE::NiAVObject* groundNode = nullptr;
	if (RE::Actor* actor = a_context.getActor())
		groundNode = actor->Get3D();
	const int groundDriver = addDriver(groundNode ? groundNode : meshRoot);

	s.shapes = &shapes;
	s.hairShape = hairShape;
	s.skeleton = skeleton;
	s.meshRoot = meshRoot;
	s.hairMesh = hairMesh;
	s.headNode = headNode;
	s.neckNode = neckNode;
	s.spineNode = spineNode;
	s.leftClavicleNode = leftClavicleNode;
	s.rightClavicleNode = rightClavicleNode;
	s.leftUpperArmNode = leftUpperArmNode;
	s.rightUpperArmNode = rightUpperArmNode;
	s.leftForearmNode = leftForearmNode;
	s.rightForearmNode = rightForearmNode;
	s.lowerTorsoNode = lowerTorsoNode;
	s.middleSpineNode = middleSpineNode;
	s.headDriver = headDriver;
	s.neckDriver = neckDriver;
	s.spineDriver = spineDriver;
	s.leftClavicleDriver = leftClavicleDriver;
	s.rightClavicleDriver = rightClavicleDriver;
	s.leftArmDriver = leftArmDriver;
	s.rightArmDriver = rightArmDriver;
	s.lowerTorsoDriver = lowerTorsoDriver;
	s.groundDriver = groundDriver;
	return true;
}

bool SmpPhysicsFeature::BuildHairSpheres(CollisionBuildState& s, std::string& a_reason) const
{
	const hdt::IMeshClaimContext& a_context = s.context;
	const SmpPhysics::SmpRig& a_rig = s.rig;
	ClaimCollision& a_collision = s.collision;
	const SmpPhysics::RigCollision& shapes = *s.shapes;
	const RigShape* hairShape = s.hairShape;
	RE::NiAVObject* meshRoot = s.meshRoot;
	RE::NiAVObject* hairMesh = s.hairMesh;
	std::map<std::string, int> rigBoneOfName;
	for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
		const char* mergedName = a_context.getMergedBoneName(a_rig.bones[index].name.c_str());
		rigBoneOfName.insert_or_assign(a_rig.bones[index].name, static_cast<int>(index));
		if (mergedName && *mergedName)
			rigBoneOfName.insert_or_assign(mergedName, static_cast<int>(index));
	}

	const std::vector<Transform> vertexToBone(a_rig.bones.size());
	std::vector<bool> dynamicBone(a_rig.bones.size(), false);
	std::vector<float> marginMultiplier(a_rig.bones.size(), 1.0f);
	for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
		dynamicBone[index] = !a_rig.bones[index].kinematic;
		marginMultiplier[index] = a_rig.bones[index].marginMultiplier;
	}

	std::vector<SkinnedVertex> hairVertices;
	std::string readReason;
	std::uint32_t mappedSkinBones = 0;
	std::uint32_t totalSkinBones = 0;
	std::vector<const RigShape*> sphereShapes;
	for (const RigShape& shape : shapes.vertexShapes)
		sphereShapes.push_back(&shape);
	if (std::ranges::find(sphereShapes, hairShape) == sphereShapes.end())
		sphereShapes.push_back(hairShape);
	// A cloth rig declares only triangle shapes. Each piece of the outfit that skins to its bones gets spheres of its own, so
	// the pieces collide with one another as their tags say, not only the one the hair shape picked.
	const bool perShape = shapes.vertexShapes.empty();
	if (perShape) {
		for (const RigShape& shape : shapes.triangleShapes) {
			if (&shape != hairShape)
				sphereShapes.push_back(&shape);
		}
	}
	std::vector<SkinnedVertex> shapeVertices;
	std::vector<const RigShape*> boneShape(a_rig.bones.size(), nullptr);
	std::vector<const RigShape*> contributing;
	std::vector<std::vector<BoneSphere>> mergedSpheres;
	for (const RigShape* sphereShape : sphereShapes) {
		const RigShape& shape = *sphereShape;
		const bool extraCloth = perShape && &shape != hairShape;
		RE::NiAVObject* mesh = &shape == hairShape ? hairMesh : FindMeshObject(a_context, meshRoot, shape.name);
		if (extraCloth && (!mesh || !SkinsToDynamicBone(mesh, a_rig, a_context)))
			continue;
		if (!mesh) {
			logger::warn("[GPU SMP Physics] per-vertex-shape '{}' names no mesh below the actor, so it adds no collision spheres", shape.name);
			continue;
		}
		shapeVertices.clear();
		std::vector<SkinnedVertex>& gathered = perShape ? shapeVertices : hairVertices;
		std::vector<SkinnedVertex> vertices;
		if (!ReadShape(mesh, true, vertices, readReason)) {
			if (&shape == hairShape) {
				a_reason = std::format("the hair mesh '{}' {}", shape.name, readReason);
				return false;
			}
			logger::warn("[GPU SMP Physics] per-vertex-shape '{}' {}, so it adds no collision spheres", shape.name, readReason);
			continue;
		}
		auto* geometry = mesh->AsGeometry();
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		if (!skin || !skin->bones || !skin->skinData) {
			if (&shape == hairShape) {
				a_reason = std::format("the hair mesh '{}' has no skin bones to map its vertex indices through", shape.name);
				return false;
			}
			continue;
		}

		const std::uint32_t skinBones = std::min<std::uint32_t>(skin->skinData->GetBoneCount(), kMaximumSkinBones);
		std::vector<int> rigBoneOfSkinBone(skinBones, -1);
		std::vector<float> weightThreshold(a_rig.bones.size(), 0.0f);
		for (std::uint32_t index = 0; index < skinBones; ++index) {
			RE::NiAVObject* bone = skin->bones[index];
			if (!bone || !bone->name.c_str())
				continue;
			const auto found = rigBoneOfName.find(bone->name.c_str());
			if (found != rigBoneOfName.end()) {
				rigBoneOfSkinBone[index] = found->second;
				++mappedSkinBones;
				const std::string nodeName = Util::ToLowerAscii(bone->name.c_str());
				for (const SmpPhysics::RigShapeWeight& weight : shape.weights) {
					const char* merged = a_context.getMergedBoneName(weight.bone.c_str());
					if (nodeName == Util::ToLowerAscii(weight.bone) || (merged && *merged && nodeName == Util::ToLowerAscii(merged)))
						weightThreshold[static_cast<std::size_t>(found->second)] = weight.threshold;
				}
			}
		}
		totalSkinBones += skinBones;
		for (const SkinnedVertex& vertex : vertices) {
			for (int slot = 0; slot < 4; ++slot) {
				const int skinBone = vertex.bone[slot];
				if (skinBone < 0 || static_cast<std::size_t>(skinBone) >= rigBoneOfSkinBone.size())
					continue;
				const int rigBone = rigBoneOfSkinBone[static_cast<std::size_t>(skinBone)];
				if (rigBone < 0 || vertex.weight[slot] <= weightThreshold[static_cast<std::size_t>(rigBone)])
					continue;
				SkinnedVertex local;
				local.position = ToSolverTransform(skin->skinData->GetBoneDataSkinToBone(static_cast<std::uint32_t>(skinBone))).TransformPoint(vertex.position);
				local.bone[0] = rigBone;
				local.weight[0] = vertex.weight[slot];
				gathered.push_back(local);
			}
		}
		if (perShape) {
			std::vector<std::vector<BoneSphere>> reduced = SmpPhysics::ReduceHairSpheres(shapeVertices, vertexToBone, dynamicBone, shape.margin, marginMultiplier);
			mergedSpheres.resize(reduced.size());
			bool any = false;
			for (std::size_t bone = 0; bone < reduced.size(); ++bone) {
				if (reduced[bone].empty())
					continue;
				any = true;
				mergedSpheres[bone].insert(mergedSpheres[bone].end(), reduced[bone].begin(), reduced[bone].end());
				if (bone < boneShape.size() && boneShape[bone] == nullptr)
					boneShape[bone] = &shape;
			}
			if (any)
				contributing.push_back(&shape);
		}
	}

	a_collision.spheres = perShape ? std::move(mergedSpheres) : SmpPhysics::ReduceHairSpheres(hairVertices, vertexToBone, dynamicBone, hairShape->margin, marginMultiplier);
	if (a_collision.spheres.size() != a_rig.bones.size())
		a_collision.spheres.resize(a_rig.bones.size());
	s.sphereShapes = !contributing.empty() ? contributing : std::vector<const RigShape*>{ hairShape };
	if (contributing.size() > 1)
		s.boneShape = std::move(boneShape);
	for (const std::vector<BoneSphere>& spheres : a_collision.spheres)
		a_collision.sphereCount += static_cast<int>(spheres.size());
	if (a_collision.sphereCount == 0) {
		a_reason = std::format("no hair spheres could be built for '{}' ({} of {} skin bones matched a rig bone)", hairShape->name,
			mappedSkinBones, totalSkinBones);
		return false;
	}

	s.rigBoneOfName = std::move(rigBoneOfName);
	s.dynamicBone = std::move(dynamicBone);
	return true;
}

void SmpPhysicsFeature::BuildBodyProxies(CollisionBuildState& s) const
{
	const hdt::IMeshClaimContext& a_context = s.context;
	ClaimCollision& a_collision = s.collision;
	const SmpPhysics::RigCollision& shapes = *s.shapes;
	const RigShape* hairShape = s.hairShape;
	RE::NiAVObject* meshRoot = s.meshRoot;
	RE::NiAVObject* headNode = s.headNode;
	RE::NiAVObject* neckNode = s.neckNode;
	RE::NiAVObject* spineNode = s.spineNode;
	RE::NiAVObject* leftClavicleNode = s.leftClavicleNode;
	RE::NiAVObject* rightClavicleNode = s.rightClavicleNode;
	RE::NiAVObject* leftUpperArmNode = s.leftUpperArmNode;
	RE::NiAVObject* rightUpperArmNode = s.rightUpperArmNode;
	RE::NiAVObject* leftForearmNode = s.leftForearmNode;
	RE::NiAVObject* rightForearmNode = s.rightForearmNode;
	RE::NiAVObject* lowerTorsoNode = s.lowerTorsoNode;
	RE::NiAVObject* middleSpineNode = s.middleSpineNode;
	const int headDriver = s.headDriver;
	const int neckDriver = s.neckDriver;
	const int spineDriver = s.spineDriver;
	const int leftClavicleDriver = s.leftClavicleDriver;
	const int rightClavicleDriver = s.rightClavicleDriver;
	const int leftArmDriver = s.leftArmDriver;
	const int rightArmDriver = s.rightArmDriver;
	const int lowerTorsoDriver = s.lowerTorsoDriver;
	const int groundDriver = s.groundDriver;
	const std::map<std::string, int>& rigBoneOfName = s.rigBoneOfName;
	const std::vector<bool>& dynamicBone = s.dynamicBone;
	std::string readReason;
	const RigShape* headShape = PickTriangleShape(shapes, *hairShape, "head");
	std::vector<Vec3> headVertices;
	if (headShape != nullptr) {
		if (RE::NiAVObject* headMesh = FindMeshObject(a_context, meshRoot, headShape->name)) {
			std::vector<Vec3> posed;
			if (ReadPosedPositions(headMesh, posed, readReason)) {
				const Transform worldToHead = ToSolverTransform(headNode->world).Inverse();
				headVertices.reserve(posed.size());
				for (const Vec3& vertex : posed)
					headVertices.push_back(worldToHead.TransformPoint(vertex));
			}
		}
	}
	if (headVertices.size() >= 4) {
		Collider head = SmpPhysics::FitHeadEllipsoid(headVertices, headDriver);
		if (headShape->penetration > 0.0f)
			head.penetration = headShape->penetration;
		a_collision.colliders.push_back(head);
		a_collision.colliderFits.push_back({ static_cast<int>(headVertices.size()), true });
	} else {
		Collider head = SmpPhysics::MakeDefaultHeadCollider(headDriver);
		if (headShape != nullptr && headShape->penetration > 0.0f)
			head.penetration = headShape->penetration;
		a_collision.colliders.push_back(head);
		a_collision.colliderFits.push_back({ static_cast<int>(headVertices.size()), false });
	}

	std::vector<RE::NiAVObject*> segmentNodes;
	const auto segmentIndexOf = [&segmentNodes](RE::NiAVObject* a_node) {
		if (a_node == nullptr)
			return -1;
		for (std::size_t index = 0; index < segmentNodes.size(); ++index) {
			if (segmentNodes[index] == a_node)
				return static_cast<int>(index);
		}
		segmentNodes.push_back(a_node);
		return static_cast<int>(segmentNodes.size()) - 1;
	};

	struct Segment
	{
		int nearIndex;
		int farIndex;
		int driver;
		float defaultRadius;
	};
	const std::vector<Segment> segments = {
		{ segmentIndexOf(neckNode), segmentIndexOf(headNode), neckDriver, SmpPhysics::kDefaultNeckRadius },
		{ segmentIndexOf(lowerTorsoNode), segmentIndexOf(middleSpineNode), lowerTorsoDriver, SmpPhysics::kDefaultTorsoRadius },
		{ segmentIndexOf(leftClavicleNode), segmentIndexOf(leftUpperArmNode), leftClavicleDriver, SmpPhysics::kDefaultShoulderRadius },
		{ segmentIndexOf(rightClavicleNode), segmentIndexOf(rightUpperArmNode), rightClavicleDriver, SmpPhysics::kDefaultShoulderRadius },
		{ segmentIndexOf(leftUpperArmNode), segmentIndexOf(leftForearmNode), leftArmDriver, SmpPhysics::kDefaultArmRadius },
		{ segmentIndexOf(rightUpperArmNode), segmentIndexOf(rightForearmNode), rightArmDriver, SmpPhysics::kDefaultArmRadius }
	};

	const int spineSegment = segmentIndexOf(spineNode);
	const int middleSpineSegment = segmentIndexOf(middleSpineNode);

	const std::function<bool(const RE::NiAVObject*)> isHairBone = [&rigBoneOfName, &dynamicBone](const RE::NiAVObject* a_node) {
		const char* name = a_node ? a_node->name.c_str() : nullptr;
		if (!name)
			return false;
		const auto found = rigBoneOfName.find(name);
		return found != rigBoneOfName.end() && dynamicBone[static_cast<std::size_t>(found->second)];
	};

	BodyScanInput scan;
	scan.root = meshRoot;
	scan.segmentNodes = segmentNodes;
	scan.isHairBone = isHairBone;
	scan.measure = true;
	for (const RigShape& shape : shapes.vertexShapes)
		scan.hairNames.push_back(shape.name);
	for (const RigShape& shape : shapes.triangleShapes)
		scan.hairNames.push_back(shape.name);
	if (RE::NiAVObject* model = a_context.getModel()) {
		if (const char* modelName = model->name.c_str())
			scan.hairNames.emplace_back(modelName);
	}
	BodyScanResult scanned;
	ScanBody(scan, scanned);
	a_collision.bodySignature = scanned.signature;
	a_collision.hairMeshNames = scan.hairNames;

	std::vector<SmpPhysics::SegmentVertex> measured;
	for (const Segment& segment : segments) {
		if (segment.nearIndex < 0 || segment.farIndex < 0 || segment.driver < 0) {
			++a_collision.defaulted;
			continue;
		}
		const RE::NiAVObject* farNode = segmentNodes[static_cast<std::size_t>(segment.farIndex)];
		const Transform frame = ToSolverTransform(a_collision.drivers[static_cast<std::size_t>(segment.driver)]->world).Inverse();
		const Vec3 endA{};
		const Vec3 endB = frame.TransformPoint(ToSolverTransform(farNode->world).translation);

		measured.clear();
		measured.reserve(scanned.samples.size());
		for (const BodySample& sample : scanned.samples)
			measured.push_back({ frame.TransformPoint(sample.world),
				sample.segmentBone == segment.nearIndex });

		const SmpPhysics::SegmentFit fit = SmpPhysics::FitSegmentRadius(measured, endA, endB, segment.defaultRadius);
		a_collision.colliders.push_back(SmpPhysics::MakeDefaultCapsule(endA, endB, fit.radius, segment.driver));
		a_collision.colliderFits.push_back({ fit.vertexCount, fit.fitted });
	}

	if (spineNode != nullptr && spineDriver >= 0) {
		const Transform frame = ToSolverTransform(a_collision.drivers[static_cast<std::size_t>(spineDriver)]->world).Inverse();
		std::vector<Vec3> torso;
		for (const BodySample& sample : scanned.samples) {
			if (sample.segmentBone >= 0 && (sample.segmentBone == spineSegment || sample.segmentBone == middleSpineSegment))
				torso.push_back(frame.TransformPoint(sample.world));
		}
		if (torso.size() >= SmpPhysics::kMinimumSegmentVertices) {
			a_collision.colliders.push_back(SmpPhysics::FitHeadEllipsoid(torso, spineDriver));
			a_collision.colliderFits.push_back({ static_cast<int>(torso.size()), true });
		} else {
			const Vec3 top = neckNode != nullptr ? frame.TransformPoint(ToSolverTransform(neckNode->world).translation) : Vec3{ 0.0f, 0.0f, kDefaultTorsoHeight };
			a_collision.colliders.push_back(SmpPhysics::MakeDefaultCapsule(Vec3{}, top, SmpPhysics::kDefaultTorsoRadius, spineDriver));
			a_collision.colliderFits.push_back({ static_cast<int>(torso.size()), false });
		}
	}

	const RigShape* groundShape = PickTriangleShape(shapes, *hairShape, "ground");
	if (groundShape != nullptr || (settings.ClothFloor && IsClothClaim(a_context))) {
		Collider ground = SmpPhysics::MakeGroundPlane(Vec3{}, Vec3{ 0.0f, 0.0f, 1.0f }, groundDriver);
		if (groundShape != nullptr && groundShape->penetration > 0.0f)
			ground.penetration = groundShape->penetration;
		a_collision.colliders.push_back(ground);
		a_collision.colliderFits.push_back({ 0, true });
	}

	s.hairNames = scan.hairNames;
}

void SmpPhysicsFeature::BuildRigSurfaces(CollisionBuildState& s) const
{
	const hdt::IMeshClaimContext& a_context = s.context;
	ClaimCollision& a_collision = s.collision;
	const SmpPhysics::RigCollision& shapes = *s.shapes;
	const RigShape* hairShape = s.hairShape;
	RE::NiAVObject* meshRoot = s.meshRoot;
	std::map<const RE::NiAVObject*, int>& driverOfNode = s.driverOfNode;
	for (std::size_t index = 0; index < a_collision.drivers.size(); ++index)
		driverOfNode.emplace(a_collision.drivers[index].get(), static_cast<int>(index));
	std::vector<RigShape> surfaceShapes = shapes.triangleShapes;
	if (const RE::Actor* actor = a_context.getActor()) {
		const std::scoped_lock lock(stateMutex);
		if (const auto known = bodyShapes.find(actor->GetFormID()); known != bodyShapes.end()) {
			for (const BodyShape& body : known->second)
				surfaceShapes.push_back(body.shape);
		}
	}

	std::vector<RE::NiAVObject*>& rigSurfaceMeshes = s.rigSurfaceMeshes;
	for (const RigShape& shape : surfaceShapes) {
		if (shape.tag.empty())
			continue;
		const bool hairAllows = shape.name != hairShape->name && SmpPhysics::ShapesCanCollide(*hairShape, shape);
		const bool otherAllows = std::ranges::any_of(s.sphereShapes, [&shape](const RigShape* a_sphereShape) { return a_sphereShape->name != shape.name && SmpPhysics::ShapesCanCollide(*a_sphereShape, shape); });
		if (!hairAllows && !otherAllows)
			continue;
		RE::NiAVObject* surfaceMesh = FindMeshObject(a_context, meshRoot, shape.name);
		SkinnedShape skinned;
		std::string surfaceReason;
		if (!surfaceMesh || !ReadSkinnedShape(surfaceMesh, skinned, surfaceReason)) {
			logger::debug("[GPU SMP Physics] collision surface '{}' not used: {}", shape.name, surfaceMesh ? surfaceReason : std::string("no such mesh below the actor"));
			continue;
		}
		rigSurfaceMeshes.push_back(surfaceMesh);
		SmpPhysics::SkinnedSurface surface;
		surface.name = shape.name;
		surface.margin = shape.margin;
		surface.contactPenetration = shape.penetration;
		surface.deepPush = s.sphereShapes.size() > 1;
		surface.clothOnly = !hairAllows;
		surface.vertices.reserve(skinned.positions.size());
		for (std::size_t vertex = 0; vertex < skinned.positions.size(); ++vertex)
			surface.vertices.push_back(VertexOf(skinned, vertex));
		s.BindSurfaceDrivers(skinned, surface);

		std::vector<float> threshold(skinned.skinBones.size(), 0.0f);
		for (std::size_t bone = 0; bone < skinned.skinBones.size(); ++bone) {
			RE::NiAVObject* node = skinned.skinBones[bone];
			if (!node)
				continue;
			const std::string nodeName = Util::ToLowerAscii(node->name.c_str() ? node->name.c_str() : "");
			for (const SmpPhysics::RigShapeWeight& weight : shape.weights) {
				const char* merged = a_context.getMergedBoneName(weight.bone.c_str());
				if (nodeName == Util::ToLowerAscii(weight.bone) || (merged && *merged && nodeName == Util::ToLowerAscii(merged)))
					threshold[bone] = weight.threshold;
			}
		}
		const std::vector<std::uint32_t> triangles(skinned.triangles.begin(), skinned.triangles.end());
		surface.indices = SmpPhysics::KeepWeightedTriangles(surface.vertices, triangles, threshold);
		if (surface.indices.empty())
			continue;
		a_collision.surfaces.push_back(std::move(surface));
		s.rigSurfaceShapes.push_back(shape);
	}

	if (!a_collision.surfaces.empty()) {
		std::size_t kept = 0;
		for (std::size_t index = 0; index < a_collision.colliders.size(); ++index) {
			if (a_collision.colliders[index].type != SmpPhysics::Collider::Type::Plane)
				continue;
			a_collision.colliders[kept] = a_collision.colliders[index];
			a_collision.colliderFits[kept] = a_collision.colliderFits[index];
			++kept;
		}
		a_collision.colliders.resize(kept);
		a_collision.colliderFits.resize(kept);
	}
}

void SmpPhysicsFeature::BuildArmorSurfaces(CollisionBuildState& s) const
{
	const hdt::IMeshClaimContext& a_context = s.context;
	const SmpPhysics::SmpRig& a_rig = s.rig;
	const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes = s.boneNodes;
	ClaimCollision& a_collision = s.collision;
	Vec3 hairAnchor{};
	const RE::Actor* wornBy = a_context.getActor();
	const RE::BipedAnim* biped = wornBy != nullptr ? wornBy->GetBiped().get() : nullptr;
	std::vector<SmpPhysics::SkinnedSurface> armorSurfaces;
	const bool hairAcceptsUntagged = s.hairShape == nullptr || SmpPhysics::AllowsCollisionWithTag(*s.hairShape, std::string());
	if (hairAcceptsUntagged && biped != nullptr && FindHairAnchor(a_rig, a_boneNodes, a_collision.drivers, hairAnchor)) {
		const RE::TESObjectARMO* actorSkin = wornBy->GetSkin();
		for (std::uint32_t slot = 0; slot < RE::BIPED_OBJECTS::kTotal; ++slot) {
			if (IsSkippedArmorSlot(slot))
				continue;
			const RE::BIPOBJECT& worn = biped->objects[slot];
			const RE::TESObjectARMO* wornArmor = worn.item != nullptr ? worn.item->As<RE::TESObjectARMO>() : nullptr;
			// The bare body is the surface hair rests on when nothing covers the body slot; elsewhere the skin hides under what is worn.
			const bool bareBody = wornArmor != nullptr && wornArmor == actorSkin && slot == static_cast<std::uint32_t>(RE::BIPED_OBJECTS::kBody);
			if (wornArmor == nullptr || (wornArmor == actorSkin && !bareBody) || !worn.partClone)
				continue;

			std::vector<RE::NiPointer<RE::NiAVObject>> pending{ worn.partClone };
			while (!pending.empty()) {
				const RE::NiPointer<RE::NiAVObject> held = std::move(pending.back());
				pending.pop_back();
				RE::NiAVObject* mesh = held.get();
				if (mesh == nullptr)
					continue;
				if (RE::NiNode* node = mesh->AsNode()) {
					for (auto& child : node->GetChildren())
						pending.emplace_back(child.get());
				}
				if (!IsSkinnedGeometry(mesh) || IsNamed(mesh, s.hairNames) ||
					std::ranges::find(s.rigSurfaceMeshes, mesh) != s.rigSurfaceMeshes.end())
					continue;

				const char* meshName = mesh->name.c_str() ? mesh->name.c_str() : "";
				SkinnedShape wornShape;
				std::string wornReason;
				if (!ReadSkinnedShape(mesh, wornShape, wornReason)) {
					logger::debug("[GPU SMP Physics] worn mesh '{}' not used for collision: {}", meshName, wornReason);
					continue;
				}
				if (SkinsToDynamicBone(mesh, a_rig, a_context))
					continue;
				std::vector<Vec3> posed(wornShape.positions.size());
				for (std::size_t vertex = 0; vertex < posed.size(); ++vertex)
					posed[vertex] = Posed(wornShape, vertex);
				const std::vector<std::uint32_t> triangles(wornShape.triangles.begin(), wornShape.triangles.end());

				SmpPhysics::SkinnedSurface surface;
				surface.name = std::format("{}: {}", bareBody ? "bare body" : "worn armor", meshName);
				surface.margin = SmpPhysics::kArmorSurfaceMargin;
				surface.contactPenetration = SmpPhysics::kArmorSurfacePenetration;
				surface.deepPush = false;
				// Armor NIFs carry the body under the plate as their own mesh; hair was modelled against that skin and must not be turned away from it.
				const std::string lowerName = Util::ToLowerAscii(meshName);
				surface.skin = bareBody || lowerName.find("body") != std::string::npos || lowerName.find("torso") != std::string::npos;
				surface.vertices.reserve(wornShape.positions.size());
				for (std::size_t vertex = 0; vertex < wornShape.positions.size(); ++vertex)
					surface.vertices.push_back(VertexOf(wornShape, vertex));
				surface.indices = SmpPhysics::KeepTrianglesNear(posed, triangles, hairAnchor, SmpPhysics::kArmorSurfaceRadius);
				if (surface.indices.empty())
					continue;
				SmpPhysics::CompactSurface(surface);
				s.BindSurfaceDrivers(wornShape, surface);
				armorSurfaces.push_back(std::move(surface));
			}
		}
	}
	if (armorSurfaces.size() > SmpPhysics::kMaxArmorSurfaces) {
		std::partial_sort(armorSurfaces.begin(), armorSurfaces.begin() + static_cast<std::ptrdiff_t>(SmpPhysics::kMaxArmorSurfaces),
			armorSurfaces.end(), [](const SmpPhysics::SkinnedSurface& a_left, const SmpPhysics::SkinnedSurface& a_right) {
				return a_left.indices.size() > a_right.indices.size();
			});
		armorSurfaces.resize(SmpPhysics::kMaxArmorSurfaces);
	}
	for (SmpPhysics::SkinnedSurface& surface : armorSurfaces)
		a_collision.surfaces.push_back(std::move(surface));
}

bool SmpPhysicsFeature::IsClothClaim(const hdt::IMeshClaimContext& a_context)
{
	const RE::Actor* actor = a_context.getActor();
	const RE::NiAVObject* model = a_context.getModel();
	const RE::BipedAnim* biped = actor != nullptr ? actor->GetBiped().get() : nullptr;
	if (a_context.getKind() != hdt::MeshKind::Armor || biped == nullptr || model == nullptr)
		return false;
	for (std::uint32_t slot = 0; slot < RE::BIPED_OBJECTS::kTotal; ++slot) {
		const RE::NiAVObject* root = biped->objects[slot].partClone.get();
		if (root == nullptr || IsSkippedArmorSlot(slot) || slot == static_cast<std::uint32_t>(RE::BIPED_OBJECTS::kBody))
			continue;
		for (const RE::NiAVObject* node = model; node != nullptr; node = node->parent) {
			if (node == root)
				return true;
		}
	}
	return false;
}

void SmpPhysicsFeature::RegisterClothMeshes(RE::FormID a_actorFormID, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_geometries)
{
	std::vector<std::string>& known = clothMeshNames[a_actorFormID];
	for (const RE::NiPointer<RE::NiAVObject>& geometry : a_geometries) {
		if (geometry && geometry->name.c_str() && std::ranges::find(known, std::string(geometry->name.c_str())) == known.end())
			known.emplace_back(geometry->name.c_str());
	}
	std::ranges::sort(known);
}

std::uint64_t SmpPhysicsFeature::HashClothMeshes(RE::FormID a_actorFormID, std::uint64_t a_signature) const
{
	if (const auto known = clothMeshNames.find(a_actorFormID); known != clothMeshNames.end()) {
		for (const std::string& name : known->second)
			a_signature = Util::Fnv1a64(name, a_signature);
	}
	return a_signature;
}

void SmpPhysicsFeature::BuildClothSurfaces(CollisionBuildState& s) const
{
	const RE::Actor* actor = s.context.getActor();
	if (!settings.ClothCollision || actor == nullptr || s.rig.collision.vertexShapes.empty() || IsClothClaim(s.context))
		return;
	std::vector<std::string> clothNames;
	{
		const std::scoped_lock lock(stateMutex);
		s.collision.bodySignature = HashClothMeshes(actor->GetFormID(), s.collision.bodySignature);
		if (const auto known = clothMeshNames.find(actor->GetFormID()); known != clothMeshNames.end())
			clothNames = known->second;
	}
	Vec3 hairAnchor{};
	if (clothNames.empty() || !FindHairAnchor(s.rig, s.boneNodes, s.collision.drivers, hairAnchor))
		return;
	std::vector<RE::NiAVObject*> cloth;
	for (const std::string& name : clothNames) {
		RE::NiAVObject* mesh = FindMeshObject(s.context, s.meshRoot, name);
		if (mesh && IsSkinnedGeometry(mesh) && std::ranges::find(cloth, mesh) == cloth.end() && !IsNamed(mesh, s.hairNames))
			cloth.push_back(mesh);
	}
	for (RE::NiAVObject* mesh : cloth) {
		SkinnedShape clothShape;
		std::string clothReason;
		if (!ReadSkinnedShape(mesh, clothShape, clothReason)) {
			logger::debug("[GPU SMP Physics] cloth mesh '{}' not used for collision: {}", mesh->name.c_str(), clothReason);
			continue;
		}
		std::vector<Vec3> posed(clothShape.positions.size());
		for (std::size_t vertex = 0; vertex < posed.size(); ++vertex)
			posed[vertex] = Posed(clothShape, vertex);
		const std::vector<std::uint32_t> triangles(clothShape.triangles.begin(), clothShape.triangles.end());

		SmpPhysics::SkinnedSurface surface;
		surface.name = std::format("cloth: {}", mesh->name.c_str() ? mesh->name.c_str() : "");
		surface.margin = SmpPhysics::kClothSurfaceMargin;
		surface.contactPenetration = 0.0f;
		surface.deepPush = false;
		surface.vertices.reserve(clothShape.positions.size());
		for (std::size_t vertex = 0; vertex < clothShape.positions.size(); ++vertex)
			surface.vertices.push_back(VertexOf(clothShape, vertex));
		surface.indices = SmpPhysics::KeepTrianglesNear(posed, triangles, hairAnchor, SmpPhysics::kArmorSurfaceRadius);
		if (surface.indices.empty())
			continue;
		SmpPhysics::CompactSurface(surface);
		s.BindSurfaceDrivers(clothShape, surface);
		s.collision.surfaces.push_back(std::move(surface));
	}
}

void SmpPhysicsFeature::FinishCollision(CollisionBuildState& s) const
{
	const SmpPhysics::SmpRig& a_rig = s.rig;
	const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes = s.boneNodes;
	ClaimCollision& a_collision = s.collision;
	for (const ClaimCollision::ColliderFit& fit : a_collision.colliderFits) {
		if (fit.fitted)
			++a_collision.fitted;
		else
			++a_collision.defaulted;
	}

	// The nodes still hold the rest pose the spheres were reduced in, which is the only moment a sphere's authored
	// depth inside the body can be told from the solver having pushed it there.
	std::vector<SmpPhysics::Transform> boneRestWorld(a_rig.bones.size());
	for (std::size_t index = 0; index < boneRestWorld.size(); ++index)
		boneRestWorld[index] = s.restPose && index < s.restPose->size() ? (*s.restPose)[index] : ToSolverTransform(a_boneNodes[index]->world);
	std::vector<SmpPhysics::Transform> driverWorld(a_collision.drivers.size());
	for (std::size_t index = 0; index < driverWorld.size(); ++index)
		driverWorld[index] = ToSolverTransform(a_collision.drivers[index]->world);
	SmpPhysics::ComputeRestExemptions(a_collision.spheres, boneRestWorld, a_collision.colliders, driverWorld);
	for (const std::vector<BoneSphere>& spheres : a_collision.spheres) {
		for (const BoneSphere& sphere : spheres) {
			if (sphere.exemptMask != 0u)
				++a_collision.exemptSpheres;
		}
	}
}

bool SmpPhysicsFeature::BuildClaimCollision(const hdt::IMeshClaimContext& a_context, const SmpPhysics::SmpRig& a_rig,
	const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes, std::vector<SmpPhysics::Transform>& a_restPose, ClaimCollision& a_collision, std::string& a_reason) const
{
	CollisionBuildState state{ a_context, a_rig, a_boneNodes, a_collision };
	if (!ResolveCollisionInputs(state, a_reason) || !BuildHairSpheres(state, a_reason))
		return false;
	BuildBodyProxies(state);
	BuildRigSurfaces(state);
	BuildArmorSurfaces(state);
	BuildClothSurfaces(state);
	if (!state.boneShape.empty()) {
		std::vector<const RigShape*> surfaceShapes(a_collision.surfaces.size(), nullptr);
		for (std::size_t index = 0; index < state.rigSurfaceShapes.size() && index < surfaceShapes.size(); ++index)
			surfaceShapes[index] = &state.rigSurfaceShapes[index];
		a_collision.boneSurfaceMask.assign(a_rig.bones.size(), 0xFFFFFFFFu);
		for (std::size_t bone = 0; bone < state.boneShape.size(); ++bone)
			a_collision.boneSurfaceMask[bone] = SmpPhysics::AllowedSurfaceMask(state.boneShape[bone], surfaceShapes);
	}
	RetargetRest(state, a_restPose);
	FinishCollision(state);
	if (!state.boneShape.empty()) {
		// The nodes still hold the rest pose, so a sphere behind a layer here was authored inside it.
		std::vector<SmpPhysics::Transform> boneRestWorld(a_rig.bones.size());
		for (std::size_t index = 0; index < boneRestWorld.size(); ++index)
			boneRestWorld[index] = state.restPose && index < state.restPose->size() ? (*state.restPose)[index] : ToSolverTransform(a_boneNodes[index]->world);
		std::vector<SmpPhysics::Transform> driverWorld(a_collision.drivers.size());
		for (std::size_t index = 0; index < driverWorld.size(); ++index)
			driverWorld[index] = ToSolverTransform(a_collision.drivers[index]->world);
		std::vector<SmpPhysics::SphereProbe> probes;
		SmpPhysics::ProbeSpheres(a_collision.spheres, a_collision.colliders, a_collision.surfaces, boneRestWorld, driverWorld, probes);
		a_collision.layerExempted = SmpPhysics::ExemptSurfacesBehind(probes, state.rigSurfaceShapes.size(), a_rig.bones.size(), a_collision.boneSurfaceMask);
	}
	std::vector<SmpPhysics::CollisionOverride> overrides;
	{
		const std::scoped_lock lock(stateMutex);
		overrides = collisionOverrides;
	}
	if (!overrides.empty()) {
		std::vector<std::string> boneShapeName(a_rig.bones.size(), state.hairShape->name);
		for (std::size_t bone = 0; bone < state.boneShape.size() && bone < boneShapeName.size(); ++bone) {
			if (state.boneShape[bone])
				boneShapeName[bone] = state.boneShape[bone]->name;
		}
		std::vector<std::string> surfaceNames;
		for (const SmpPhysics::SkinnedSurface& surface : a_collision.surfaces)
			surfaceNames.push_back(surface.name);
		SmpPhysics::ApplyCollisionOverrides(overrides, boneShapeName, surfaceNames, a_collision.spheres, a_collision.boneSurfaceMask);
	}
	return true;
}

void SmpPhysicsFeature::RetargetRest(CollisionBuildState& s, std::vector<SmpPhysics::Transform>& a_restPose) const
{
	ClaimCollision& a_collision = s.collision;
	Vec3 axis{};
	const bool drape = settings.DrapeCloth && IsClothClaim(s.context);
	if (!(drape || settings.RetargetRest) || a_collision.surfaces.empty() || !FindHairAnchor(s.rig, s.boneNodes, a_collision.drivers, axis))
		return;
	std::vector<SmpPhysics::Transform> driverWorld(a_collision.drivers.size());
	for (std::size_t index = 0; index < driverWorld.size(); ++index)
		driverWorld[index] = ToSolverTransform(a_collision.drivers[index]->world);
	const SmpPhysics::RetargetLoad load{ SmpPhysics::SolverParams{}.gravity * settings.GravityScale, settings.StiffnessScale };
	std::vector<SmpPhysics::SkinnedSurface> retargetSurfaces;
	const bool anyClothOnly = std::ranges::any_of(a_collision.surfaces, [](const SmpPhysics::SkinnedSurface& a_surface) { return a_surface.clothOnly; });
	if (anyClothOnly) {
		for (const SmpPhysics::SkinnedSurface& surface : a_collision.surfaces) {
			if (!surface.clothOnly)
				retargetSurfaces.push_back(surface);
		}
		if (retargetSurfaces.empty())
			return;
	}
	const std::vector<SmpPhysics::SkinnedSurface>& supports = anyClothOnly ? retargetSurfaces : a_collision.surfaces;
	SmpPhysics::RestRetarget retarget = drape ? SmpPhysics::DrapeRestOntoSupports(s.rig, a_restPose, a_collision.spheres, supports, driverWorld, axis, load) :
	                                            SmpPhysics::RetargetRestToSurfaces(s.rig, a_restPose, a_collision.spheres, supports, driverWorld, axis, load);
	a_collision.retargetBehind = retarget.behindSpheres;
	a_collision.retargetSagMaxMove = retarget.sagMaxMove;
	a_collision.sagPose = std::move(retarget.sagPose);
	a_collision.retargetResidual = retarget.residualSpheres;
	a_collision.retargetedJoints = retarget.retargetedJoints;
	a_collision.retargetKeptAuthored = retarget.keptAuthored;
	a_collision.retargetMaxAngle = retarget.maxAngle;
	a_collision.retargetMaxBoneMove = retarget.maxBoneMove;
	if (retarget.retargetedJoints == 0)
		return;
	a_restPose = std::move(retarget.restPose);
	s.restPose = &a_restPose;
}

bool SmpPhysicsFeature::WornMeshesChanged(Claim& a_claim, float a_frameDelta)
{
	if ((a_claim.colliders.empty() && a_claim.surfaces.empty()) || a_claim.actorFormID == 0)
		return false;
	a_claim.refitCheckElapsed += a_frameDelta;
	if (a_claim.refitCheckElapsed < kRefitCheckSeconds)
		return false;
	a_claim.refitCheckElapsed = 0.0;

	const auto* actor = RE::TESForm::LookupByID<RE::Actor>(a_claim.actorFormID);
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root)
		return false;

	BodyScanInput scan;
	scan.root = root;
	scan.hairNames = a_claim.hairMeshNames;
	BodyScanResult scanned;
	ScanBody(scan, scanned);
	std::uint64_t signature = scanned.signature;
	if (settings.ClothCollision && !a_claim.cloth && a_claim.parsed && !a_claim.parsed->rig.collision.vertexShapes.empty())
		signature = HashClothMeshes(a_claim.actorFormID, signature);
	if (signature == a_claim.bodySignature)
		return false;

	// The signature is adopted whether or not the caller manages to re-run claiming, so one equipment change
	// cannot ask hdtSMP64 to reload the actor again on every frame the meshes stay different from the proxies.
	a_claim.bodySignature = signature;
	return true;
}

namespace SmpPhysics
{
	bool ReadCardVertices(RE::NiAVObject* a_geometry, std::vector<SkinnedVertex>& a_vertices, std::string& a_reason)
	{
		return ReadShape(a_geometry, true, a_vertices, a_reason);
	}

	bool ReadCardTriangles(RE::NiAVObject* a_geometry, std::vector<std::uint16_t>& a_triangles, std::string& a_reason)
	{
		auto* geometry = a_geometry ? a_geometry->AsGeometry() : nullptr;
		RE::NiSkinInstance* skin = geometry ? geometry->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		RE::NiSkinPartition* partition = skin ? skin->skinPartition.get() : nullptr;
		if (!partition || partition->partitions.empty()) {
			a_reason = "has no skin partition to read triangles from";
			return false;
		}
		a_triangles.clear();
		for (const RE::NiSkinPartition::Partition& entry : partition->partitions) {
			if (entry.strips != 0) {
				a_reason = "stores its triangles as strips";
				return false;
			}
			if (!entry.triList || entry.triangles == 0)
				continue;
			a_triangles.insert(a_triangles.end(), entry.triList, entry.triList + static_cast<std::size_t>(entry.triangles) * 3);
		}
		if (a_triangles.empty()) {
			a_reason = "has no triangle list";
			return false;
		}
		return true;
	}
}
