#include "Collision.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>

namespace SmpPhysics
{
	namespace
	{
		constexpr float kMinimumSphereRadius = 1e-4f;
		constexpr int kMaximumJacobiSweeps = 24;
		constexpr float kPercentile = 0.95f;
		constexpr std::size_t kMinimumEllipsoidVertices = 4;

		/** @brief Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations. Eigenvectors come back as columns. */
		void JacobiEigen(const float (&a_matrix)[3][3], float (&a_vectors)[3][3], float (&a_values)[3]) noexcept
		{
			float matrix[3][3];
			for (int row = 0; row < 3; ++row) {
				for (int column = 0; column < 3; ++column)
					matrix[row][column] = a_matrix[row][column];
			}
			float vectors[3][3] = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

			for (int sweep = 0; sweep < kMaximumJacobiSweeps; ++sweep) {
				const float offDiagonal = std::abs(matrix[0][1]) + std::abs(matrix[0][2]) + std::abs(matrix[1][2]);
				const float diagonal = std::abs(matrix[0][0]) + std::abs(matrix[1][1]) + std::abs(matrix[2][2]);
				if (offDiagonal <= 1e-9f * (diagonal + 1e-30f))
					break;
				for (int first = 0; first < 2; ++first) {
					for (int second = first + 1; second < 3; ++second) {
						if (std::abs(matrix[first][second]) < 1e-24f)
							continue;
						const float theta = (matrix[second][second] - matrix[first][first]) /
						                    (2.0f * matrix[first][second]);
						const float sign = theta >= 0.0f ? 1.0f : -1.0f;
						const float tangent = sign / (std::abs(theta) + std::sqrt(theta * theta + 1.0f));
						const float cosine = 1.0f / std::sqrt(tangent * tangent + 1.0f);
						const float sine = tangent * cosine;
						for (int index = 0; index < 3; ++index) {
							const float left = matrix[index][first];
							const float right = matrix[index][second];
							matrix[index][first] = cosine * left - sine * right;
							matrix[index][second] = sine * left + cosine * right;
						}
						for (int index = 0; index < 3; ++index) {
							const float left = matrix[first][index];
							const float right = matrix[second][index];
							matrix[first][index] = cosine * left - sine * right;
							matrix[second][index] = sine * left + cosine * right;
						}
						for (int index = 0; index < 3; ++index) {
							const float left = vectors[index][first];
							const float right = vectors[index][second];
							vectors[index][first] = cosine * left - sine * right;
							vectors[index][second] = sine * left + cosine * right;
						}
					}
				}
			}

			for (int index = 0; index < 3; ++index) {
				a_values[index] = matrix[index][index];
				for (int component = 0; component < 3; ++component)
					a_vectors[component][index] = vectors[component][index];
			}
		}

		float PercentileOfSorted(std::vector<float>& a_sorted, float a_fraction) noexcept
		{
			if (a_sorted.empty())
				return 0.0f;
			std::sort(a_sorted.begin(), a_sorted.end());
			std::size_t index = static_cast<std::size_t>(a_fraction * static_cast<float>(a_sorted.size() - 1));
			if (index >= a_sorted.size())
				index = a_sorted.size() - 1;
			return a_sorted[index];
		}

		/** @return The distance from a_point to the segment, and the closest point on it. */
		float DistanceToSegment(const Vec3& a_point, const Vec3& a_start, const Vec3& a_end, Vec3& a_closest) noexcept
		{
			const Vec3 direction = a_end - a_start;
			const float lengthSquared = Dot(direction, direction);
			float amount = 0.0f;
			if (lengthSquared > kDirectionEpsilon)
				amount = std::clamp(Dot(a_point - a_start, direction) / lengthSquared, 0.0f, 1.0f);
			a_closest = a_start + direction * amount;
			return Length(a_point - a_closest);
		}

		/** @brief A unit vector perpendicular to a_axis, so a contact at a capsule's axis still has a usable normal. */
		Vec3 AnyPerpendicular(const Vec3& a_axis) noexcept
		{
			if (Length(a_axis) <= kDirectionEpsilon)
				return { 0.0f, 0.0f, 1.0f };
			const Vec3 reference = std::abs(a_axis.z) > 0.9f ? Vec3{ 1.0f, 0.0f, 0.0f } : Vec3{ 0.0f, 0.0f, 1.0f };
			return Normalized(Cross(a_axis, reference));
		}

		/** @brief Signed distance from a_pointWorld to an ellipsoid, with the outward unit normal. */
		float EllipsoidDistance(const Collider& a_collider, const Transform& a_world, const Vec3& a_pointWorld,
			Vec3& a_outward) noexcept
		{
			const Quat rotation = (a_world.rotation * a_collider.orientation).Normalized();
			const Vec3 centre = a_world.TransformPoint(a_collider.center);
			const Vec3 local = Conjugate(rotation).Rotate(a_pointWorld - centre);
			const float halfExtents[3] = { std::max(a_collider.halfExtents.x, kMinimumHalfExtent),
				std::max(a_collider.halfExtents.y, kMinimumHalfExtent),
				std::max(a_collider.halfExtents.z, kMinimumHalfExtent) };
			const Vec3 scaled{ local.x / halfExtents[0], local.y / halfExtents[1], local.z / halfExtents[2] };
			const float radiusScale = Length(scaled);
			if (radiusScale > kDirectionEpsilon) {
				a_outward = rotation.Rotate(Normalized(local));
				return Length(local) * (radiusScale - 1.0f) / radiusScale;
			}
			int smallest = 0;
			for (int axis = 1; axis < 3; ++axis) {
				if (halfExtents[axis] < halfExtents[smallest])
					smallest = axis;
			}
			a_outward = rotation.Rotate(UnitAxis(smallest));
			return -halfExtents[smallest];
		}

		float CapsuleDistance(const Collider& a_collider, const Transform& a_world, const Vec3& a_pointWorld,
			Vec3& a_outward) noexcept
		{
			const Vec3 start = a_world.TransformPoint(a_collider.segmentA);
			const Vec3 end = a_world.TransformPoint(a_collider.segmentB);
			Vec3 closest;
			const float distance = DistanceToSegment(a_pointWorld, start, end, closest);
			const Vec3 offset = a_pointWorld - closest;
			a_outward = Normalized(offset);
			if (Length(a_outward) <= kDirectionEpsilon)
				a_outward = AnyPerpendicular(Normalized(end - start));
			return distance - a_collider.radius;
		}

		float PlaneDistance(const Collider& a_collider, const Transform& a_world, const Vec3& a_pointWorld,
			Vec3& a_outward) noexcept
		{
			a_outward = Normalized(a_world.rotation.Rotate(a_collider.normal));
			if (Length(a_outward) <= kDirectionEpsilon)
				a_outward = { 0.0f, 0.0f, 1.0f };
			return Dot(a_pointWorld - a_world.TransformPoint(a_collider.center), a_outward);
		}

		/** @brief Signed distance from a_pointWorld to a collider of any kind, with the outward unit normal. */
		float ColliderDistance(const Collider& a_collider, const Transform& a_world, const Vec3& a_pointWorld,
			Vec3& a_outward) noexcept
		{
			switch (a_collider.type) {
			case Collider::Type::Capsule:
				return CapsuleDistance(a_collider, a_world, a_pointWorld, a_outward);
			case Collider::Type::Plane:
				return PlaneDistance(a_collider, a_world, a_pointWorld, a_outward);
			case Collider::Type::Ellipsoid:
				break;
			}
			return EllipsoidDistance(a_collider, a_world, a_pointWorld, a_outward);
		}
	}

	Vec3 SkinVertexToWorld(const SkinnedVertex& a_vertex, const std::vector<Transform>& a_boneToWorld,
		const std::vector<Transform>& a_skinToBone) noexcept
	{
		Vec3 sum{};
		float total = 0.0f;
		for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
			const int bone = a_vertex.bone[slot];
			if (bone < 0 || static_cast<std::size_t>(bone) >= a_boneToWorld.size() ||
				static_cast<std::size_t>(bone) >= a_skinToBone.size())
				continue;
			const float weight = a_vertex.weight[slot];
			if (!(weight > 0.0f))
				continue;
			const Transform skin = a_boneToWorld[static_cast<std::size_t>(bone)] * a_skinToBone[static_cast<std::size_t>(bone)];
			sum = sum + skin.TransformPoint(a_vertex.position) * weight;
			total += weight;
		}
		if (!(total > 0.0f))
			return a_vertex.position;
		return sum / total;
	}

	int DominantBone(const SkinnedVertex& a_vertex, int a_boneCount) noexcept
	{
		const auto owner = Util::DominantSkinBone(a_vertex.bone, a_vertex.weight, static_cast<std::uint32_t>(std::max(a_boneCount, 0)));
		return owner ? static_cast<int>(*owner) : -1;
	}

	std::vector<std::vector<BoneSphere>> ReduceHairSpheres(const std::vector<SkinnedVertex>& a_vertices,
		const std::vector<Transform>& a_meshToBone, const std::vector<bool>& a_dynamicBone, float a_margin,
		const std::vector<float>& a_marginMultiplier, const std::vector<VertexBlend>& a_blend)
	{
		std::vector<std::vector<BoneSphere>> result(a_meshToBone.size());
		std::vector<std::vector<BoneSphere>> clusters(a_meshToBone.size());
		float margin = a_margin;
		if (!std::isfinite(margin) || margin < 0.0f)
			margin = 0.0f;

		for (std::size_t vertexIndex = 0; vertexIndex < a_vertices.size(); ++vertexIndex) {
			const SkinnedVertex& vertex = a_vertices[vertexIndex];
			if (!IsFiniteVector(vertex.position))
				continue;
			for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
				const int bone = vertex.bone[slot];
				if (bone < 0 || static_cast<std::size_t>(bone) >= a_meshToBone.size() || !(vertex.weight[slot] > FLT_EPSILON))
					continue;
				const std::size_t boneIndex = static_cast<std::size_t>(bone);
				if (boneIndex >= a_dynamicBone.size() || !a_dynamicBone[boneIndex] || !IsFiniteTransform(a_meshToBone[boneIndex]))
					continue;
				BoneSphere sphere;
				sphere.centerLocal = a_meshToBone[boneIndex].TransformPoint(vertex.position);
				if (vertexIndex < a_blend.size()) {
					const VertexBlend& blend = a_blend[vertexIndex];
					sphere.weight = blend.ownerWeight;
					if (blend.marginMultiplier >= 0.0f)
						sphere.radius = std::max(margin * blend.marginMultiplier, kMinimumSphereRadius);
					sphere.flexible = blend.flexible;
					for (std::size_t other = 0; other < blend.others.size(); ++other) {
						const SphereInfluence& influence = blend.others[other];
						if (influence.bone < 0 || static_cast<std::size_t>(influence.bone) >= a_meshToBone.size() || !(influence.weight > 0.0f) || !IsFiniteVector(influence.local))
							continue;
						sphere.others[other] = { influence.bone, a_meshToBone[static_cast<std::size_t>(influence.bone)].TransformPoint(influence.local), influence.weight };
					}
				}
				clusters[boneIndex].push_back(sphere);
			}
		}

		for (std::size_t bone = 0; bone < clusters.size(); ++bone) {
			const std::vector<BoneSphere>& cluster = clusters[bone];
			const std::size_t wanted = std::min<std::size_t>(kMaxContactSpheresPerBone, cluster.size());
			result[bone].reserve(wanted);
			const float multiplier = bone < a_marginMultiplier.size() && std::isfinite(a_marginMultiplier[bone]) ? std::max(a_marginMultiplier[bone], 0.0f) : 1.0f;
			const float radius = std::max(margin * multiplier, kMinimumSphereRadius);
			for (std::size_t index = 0; index < wanted; ++index) {
				BoneSphere sphere = cluster[index * cluster.size() / wanted];
				if (!(sphere.radius > 0.0f))
					sphere.radius = radius;
				result[bone].push_back(sphere);
			}
		}
		return result;
	}

	SphereBound BoundSpheres(const std::vector<BoneSphere>& a_spheres) noexcept
	{
		SphereBound bound;
		if (a_spheres.empty())
			return bound;
		Vec3 sum{};
		for (const BoneSphere& sphere : a_spheres)
			sum = sum + sphere.centerLocal;
		bound.centre = sum / static_cast<float>(a_spheres.size());
		for (const BoneSphere& sphere : a_spheres)
			bound.radius = std::max(bound.radius, Length(sphere.centerLocal - bound.centre) + sphere.radius);
		return bound;
	}

	void ContactMerge::Add(const Contact& a_contact, const Vec3& a_position) noexcept
	{
		if (!(a_contact.depth > 0.0f))
			return;
		if (weight > 0.0f && Dot(normal, a_contact.normal) < 0.0f)
			return;
		const float squared = a_contact.depth * a_contact.depth;
		weight += squared;
		normal = normal + a_contact.normal * (squared * a_contact.depth);
		position = position + a_position * squared;
		if (a_contact.depth > deepest) {
			deepest = a_contact.depth;
			collider = a_contact.collider;
		}
	}

	float ContactMerge::Result(Vec3& a_normal, Vec3& a_position) const noexcept
	{
		const float length = Length(normal);
		if (!(weight > 0.0f) || !(length > 0.0f))
			return 0.0f;
		a_normal = normal / length;
		a_position = position / weight;
		return length / weight;
	}

	Collider FitHeadEllipsoid(const std::vector<Vec3>& a_verticesInDriver, int a_driverIndex)
	{
		if (a_verticesInDriver.size() < kMinimumEllipsoidVertices)
			return MakeDefaultHeadCollider(a_driverIndex);

		Vec3 centre{};
		for (const Vec3& vertex : a_verticesInDriver)
			centre = centre + vertex;
		centre = centre / static_cast<float>(a_verticesInDriver.size());

		float covariance[3][3] = {};
		for (const Vec3& vertex : a_verticesInDriver) {
			const Vec3 offset = vertex - centre;
			const float components[3] = { offset.x, offset.y, offset.z };
			for (int row = 0; row < 3; ++row) {
				for (int column = row; column < 3; ++column)
					covariance[row][column] += components[row] * components[column];
			}
		}
		float spread = 0.0f;
		for (int row = 0; row < 3; ++row) {
			for (int column = row; column < 3; ++column) {
				covariance[row][column] /= static_cast<float>(a_verticesInDriver.size());
				covariance[column][row] = covariance[row][column];
				spread += covariance[row][column] * covariance[row][column];
			}
		}

		if (!(spread > 0.0f) || !std::isfinite(spread))
			return MakeDefaultHeadCollider(a_driverIndex);

		Collider collider;
		collider.type = Collider::Type::Ellipsoid;
		collider.driverIndex = a_driverIndex;
		collider.center = centre;

		float vectors[3][3];
		float values[3];
		JacobiEigen(covariance, vectors, values);

		int order[3] = { 0, 1, 2 };
		std::sort(order, order + 3, [&values](int a_left, int a_right) { return values[a_left] > values[a_right]; });

		Vec3 axes[3];
		for (int axis = 0; axis < 3; ++axis) {
			const int column = order[axis];
			axes[axis] = Normalized(Vec3{ vectors[0][column], vectors[1][column], vectors[2][column] });
		}
		if (Dot(axes[2], Cross(axes[0], axes[1])) < 0.0f)
			axes[2] = -axes[2];

		float projection[3] = {};
		for (int axis = 0; axis < 3; ++axis) {
			std::vector<float> absolute;
			absolute.reserve(a_verticesInDriver.size());
			float largest = 0.0f;
			for (const Vec3& vertex : a_verticesInDriver) {
				const float projected = std::abs(Dot(vertex - centre, axes[axis]));
				absolute.push_back(projected);
				if (projected > largest)
					largest = projected;
			}
			const float percentile = PercentileOfSorted(absolute, kPercentile);
			float halfExtent = std::min(percentile, largest);
			if (halfExtent < kMinimumHalfExtent)
				halfExtent = kMinimumHalfExtent;
			projection[axis] = halfExtent;
		}

		const float matrix[3][3] = {
			{ axes[0].x, axes[1].x, axes[2].x },
			{ axes[0].y, axes[1].y, axes[2].y },
			{ axes[0].z, axes[1].z, axes[2].z }
		};
		collider.orientation = FromRotationMatrix(matrix);
		collider.halfExtents = { projection[0], projection[1], projection[2] };
		if (!IsFiniteVector(collider.halfExtents))
			return MakeDefaultHeadCollider(a_driverIndex);
		return collider;
	}

	Collider MakeDefaultHeadCollider(int a_driverIndex)
	{
		Collider collider;
		collider.type = Collider::Type::Ellipsoid;
		collider.driverIndex = a_driverIndex;
		collider.halfExtents = { kDefaultHeadRadius, kDefaultHeadRadius, kDefaultHeadRadius };
		return collider;
	}

	SegmentFit FitSegmentRadius(const std::vector<SegmentVertex>& a_vertices, const Vec3& a_endA, const Vec3& a_endB,
		float a_defaultRadius) noexcept
	{
		SegmentFit result;
		result.radius = a_defaultRadius;

		const Vec3 direction = a_endB - a_endA;
		const float lengthSquared = Dot(direction, direction);
		std::vector<float> distances;
		for (const SegmentVertex& vertex : a_vertices) {
			if (!vertex.skinnedToSegment || !IsFiniteVector(vertex.position))
				continue;
			const float along = lengthSquared > kDirectionEpsilon ? Dot(vertex.position - a_endA, direction) / lengthSquared : 0.0f;
			if (along < -kSegmentSlack || along > 1.0f + kSegmentSlack)
				continue;
			Vec3 closest;
			distances.push_back(DistanceToSegment(vertex.position, a_endA, a_endB, closest));
		}

		result.vertexCount = static_cast<int>(distances.size());
		if (distances.size() < kMinimumSegmentVertices)
			return result;
		const float measured = PercentileOfSorted(distances, kSegmentRadiusPercentile) + kBodyArmorMargin;
		if (!std::isfinite(measured))
			return result;
		result.radius = std::min(std::max(measured, a_defaultRadius), kMaximumCapsuleRadius);
		result.fitted = true;
		return result;
	}

	Collider MakeDefaultCapsule(const Vec3& a_endA, const Vec3& a_endB, float a_radius, int a_driverIndex)
	{
		Collider collider;
		collider.type = Collider::Type::Capsule;
		collider.driverIndex = a_driverIndex;
		collider.segmentA = a_endA;
		collider.segmentB = a_endB;
		collider.radius = a_radius;
		return collider;
	}

	Collider MakeGroundPlane(const Vec3& a_point, const Vec3& a_normal, int a_driverIndex)
	{
		Collider collider;
		collider.type = Collider::Type::Plane;
		collider.driverIndex = a_driverIndex;
		collider.center = a_point;
		const Vec3 normal = Normalized(a_normal);
		collider.normal = Length(normal) > kDirectionEpsilon ? normal : Vec3{ 0.0f, 0.0f, 1.0f };
		return collider;
	}

	Contact QueryColliders(const std::vector<Collider>& a_colliders, const std::vector<Transform>& a_driverWorld,
		const Vec3& a_pointWorld, float a_radius, std::uint32_t a_exemptMask) noexcept
	{
		Contact best;
		if (!IsFiniteVector(a_pointWorld) || !std::isfinite(a_radius))
			return best;
		for (std::size_t index = 0; index < a_colliders.size(); ++index) {
			// A collider the sphere was authored inside is not a contact at all, or its spring would fight the clamp.
			if (index < kMaximumExemptColliders && (a_exemptMask & (1u << index)) != 0u)
				continue;
			const Collider& collider = a_colliders[index];
			if (collider.driverIndex < 0 || static_cast<std::size_t>(collider.driverIndex) >= a_driverWorld.size())
				continue;
			const Transform& world = a_driverWorld[static_cast<std::size_t>(collider.driverIndex)];
			if (!IsFiniteTransform(world))
				continue;

			Vec3 outward{};
			const float distance = ColliderDistance(collider, world, a_pointWorld, outward);
			const float depth = a_radius - distance;
			if (depth > best.depth) {
				best.depth = depth;
				best.normal = outward;
				best.collider = static_cast<int>(index);
			}
		}
		return best;
	}

	void ComputeRestExemptions(std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<Transform>& a_boneRestWorld, const std::vector<Collider>& a_colliders,
		const std::vector<Transform>& a_driverWorld)
	{
		const std::size_t colliderCount = std::min(a_colliders.size(), kMaximumExemptColliders);
		for (std::size_t bone = 0; bone < a_spheres.size() && bone < a_boneRestWorld.size(); ++bone) {
			const Transform& rest = a_boneRestWorld[bone];
			for (BoneSphere& sphere : a_spheres[bone]) {
				std::uint32_t mask = 0u;
				if (IsFiniteTransform(rest) && IsFiniteVector(sphere.centerLocal) && std::isfinite(sphere.radius)) {
					const Vec3 centre = rest.TransformPoint(sphere.centerLocal);
					for (std::size_t index = 0; index < colliderCount; ++index) {
						const Collider& collider = a_colliders[index];
						if (collider.driverIndex < 0 || static_cast<std::size_t>(collider.driverIndex) >= a_driverWorld.size())
							continue;
						const Transform& world = a_driverWorld[static_cast<std::size_t>(collider.driverIndex)];
						if (!IsFiniteTransform(world))
							continue;
						Vec3 outward{};
						if (sphere.radius - ColliderDistance(collider, world, centre, outward) > collider.penetration)
							mask |= 1u << index;
					}
				}
				sphere.exemptMask = mask;
			}
		}
	}
}
