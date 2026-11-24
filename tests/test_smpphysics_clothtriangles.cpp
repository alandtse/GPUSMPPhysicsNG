// Unit tests for a cloth's triangles as contact geometry (src/smp/ClothTriangles.cpp) and their packing for the
// contact shaders (src/smp/GpuRigPacking.cpp). Pure data; no game or D3D dependency.

#include "smp/ClothTriangles.h"
#include "smp/GpuRigPacking.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <cstdint>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	// tara2.xml: the dress shape's margin 0.1 and penetration -0.1 on bones of margin multiplier 3.2, against the
	// body's per-vertex shape of margin 0.55.
	constexpr float kDressMargin{ 0.1f };
	constexpr float kDressPenetration{ -0.1f };
	constexpr float kDressMultiplier{ 3.2f };
	constexpr float kBodyRadius{ 0.55f };
	constexpr float kTriangleMargin{ kDressMargin * kDressMultiplier };
	constexpr float kTrianglePenetration{ kDressPenetration * kDressMultiplier };

	// A right triangle in the z = 0 plane whose winding gives +Z as its own normal.
	constexpr Vec3 kA{ 0.0f, 0.0f, 0.0f };
	constexpr Vec3 kB{ 4.0f, 0.0f, 0.0f };
	constexpr Vec3 kC{ 0.0f, 4.0f, 0.0f };

	TriangleContact Test(const Vec3& a_vertex, float a_penetration)
	{
		return TestVertexAgainstTriangle(a_vertex, kBodyRadius, kA, kB, kC, kTriangleMargin, a_penetration);
	}

	enum RigBone : int
	{
		kBody = 0,  ///< kinematic
		kLeft = 1,
		kRight = 2,
		kRigBones = 3
	};

	/** @brief Skin bones 0 to 2 are the rig's left, right and body bones; skin bone 3 is not in the rig. */
	struct Mesh
	{
		std::vector<SkinnedVertex> vertices;
		std::vector<std::uint16_t> triangles;
		std::vector<int> rigBoneOfSkinBone{ kLeft, kRight, kBody, -1 };
		std::vector<Transform> skinToBone = std::vector<Transform>(4);
		std::vector<bool> dynamicBone{ false, true, true };
		std::vector<float> marginMultiplier{ 1.0f, 2.0f, 4.0f };
		std::vector<float> weightThreshold{ 0.0f, 0.0f, 0.0f };

		[[nodiscard]] ClothMesh View(const char* a_name) const
		{
			return { a_name, &vertices, &triangles, &rigBoneOfSkinBone, &skinToBone, &dynamicBone, &marginMultiplier, &weightThreshold, kDressMargin, kDressPenetration };
		}
	};

	SkinnedVertex Vertex(const Vec3& a_position, int a_bone0, float a_weight0, int a_bone1 = -1, float a_weight1 = 0.0f)
	{
		SkinnedVertex vertex;
		vertex.position = a_position;
		vertex.bone[0] = a_bone0;
		vertex.weight[0] = a_weight0;
		vertex.bone[1] = a_bone1;
		vertex.weight[1] = a_weight1;
		return vertex;
	}

	Mesh MakeMesh()
	{
		Mesh mesh;
		mesh.skinToBone[0].translation = { 10.0f, 0.0f, 0.0f };
		mesh.skinToBone[1].translation = { 0.0f, 20.0f, 0.0f };
		mesh.skinToBone[2].translation = { 0.0f, 0.0f, 30.0f };
		mesh.vertices = {
			Vertex({ 0.0f, 0.0f, 0.0f }, 0, 1.0f),
			Vertex({ 1.0f, 0.0f, 0.0f }, 0, 0.5f, 1, 0.5f),
			Vertex({ 2.0f, 0.0f, 0.0f }, 1, 1.0f),
			Vertex({ 2.0f, 1.0f, 0.0f }, 2, 1.0f),
			Vertex({ 3.0f, 1.0f, 0.0f }, 2, 0.9f, 1, 0.1f),
			Vertex({ 0.0f, 1.0f, 0.0f }, 3, 1.0f),
		};
		// Kept: both simulated bones on its corners; kept: the right bone on two corners; dropped: a corner nothing in
		// the rig poses; dropped: only the kinematic bone.
		mesh.triangles = { 0, 1, 2, 2, 3, 4, 0, 1, 5, 3, 3, 3 };
		return mesh;
	}
}

TEST_CASE("tara2.xml's dress holds a body vertex from 0.32 behind its plane to 0.87 in front", "[smpphysics][clothtriangles]")
{
	CHECK(kTriangleMargin == Approx(0.32f));
	CHECK(kTrianglePenetration == Approx(-0.32f));

	// A negative penetration turns the normal around, so the vertex is held on the triangle's -Z side.
	const TriangleContact inside = Test({ 1.0f, 1.0f, -0.5f }, kTrianglePenetration);
	CHECK(inside.depth == Approx(0.87f - 0.5f));
	CHECK(inside.normal.z == Approx(1.0f));
	CHECK(inside.point.x == Approx(1.0f));
	CHECK(inside.point.y == Approx(1.0f));
	CHECK(inside.point.z == Approx(0.0f).margin(1.0e-6f));
	CHECK(inside.bodyPoint.z == Approx(-0.5f + kBodyRadius));

	CHECK(Test({ 1.0f, 1.0f, -0.86f }, kTrianglePenetration).depth == Approx(0.01f).margin(1.0e-5f));
	CHECK(Test({ 1.0f, 1.0f, -(kBodyRadius + kTriangleMargin) }, kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ 1.0f, 1.0f, -0.88f }, kTrianglePenetration).depth == 0.0f);

	// The deepest contact is at the far edge of the window; past it there is no recovery.
	CHECK(Test({ 1.0f, 1.0f, -kTrianglePenetration }, kTrianglePenetration).depth == Approx(1.19f));
	CHECK(Test({ 1.0f, 1.0f, 0.33f }, kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ 1.0f, 1.0f, 5.0f }, kTrianglePenetration).depth == 0.0f);
}

TEST_CASE("The sign of the penetration picks the side of the triangle a vertex is held on", "[smpphysics][clothtriangles]")
{
	const TriangleContact front = Test({ 1.0f, 1.0f, 0.5f }, -kTrianglePenetration);
	CHECK(front.depth == Approx(0.37f));
	CHECK(front.normal.z == Approx(-1.0f));
	CHECK(front.bodyPoint.z == Approx(0.5f - kBodyRadius));
	CHECK(Test({ 1.0f, 1.0f, -0.5f }, -kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ 1.0f, 1.0f, 0.5f }, kTrianglePenetration).depth == 0.0f);
}

TEST_CASE("A triangle with no penetration holds a vertex on whichever side it is", "[smpphysics][clothtriangles]")
{
	for (const float penetration : { 0.0f, 1.0e-8f, -1.0e-8f }) {
		const TriangleContact above = Test({ 1.0f, 1.0f, 0.5f }, penetration);
		CHECK(above.depth == Approx(0.37f));
		CHECK(above.normal.z == Approx(-1.0f));
		const TriangleContact below = Test({ 1.0f, 1.0f, -0.5f }, penetration);
		CHECK(below.depth == Approx(0.37f));
		CHECK(below.normal.z == Approx(1.0f));
		CHECK(Test({ 1.0f, 1.0f, 0.88f }, penetration).depth == 0.0f);
	}
}

TEST_CASE("A vertex touches a triangle only where it projects inside it", "[smpphysics][clothtriangles]")
{
	CHECK(Test({ 1.9f, 1.9f, -0.5f }, kTrianglePenetration).depth > 0.0f);
	CHECK(Test({ 0.05f, 0.05f, -0.5f }, kTrianglePenetration).depth > 0.0f);
	CHECK(Test({ 2.1f, 2.1f, -0.5f }, kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ -0.05f, 1.0f, -0.5f }, kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ 1.0f, -0.05f, -0.5f }, kTrianglePenetration).depth == 0.0f);
	CHECK(Test({ 5.0f, 5.0f, -0.5f }, kTrianglePenetration).depth == 0.0f);
	CHECK(TestVertexAgainstTriangle({ 1.0f, 0.0f, -0.5f }, kBodyRadius, kA, kB, kB, kTriangleMargin, kTrianglePenetration).depth == 0.0f);
}

TEST_CASE("A shape's triangles are kept for the simulated bones that own a corner", "[smpphysics][clothtriangles]")
{
	const Mesh mesh = MakeMesh();
	ClothTriangleSet set;
	REQUIRE(AppendClothTriangles(set, mesh.View("dress")) == 2);

	REQUIRE(set.triangles.size() == 2);
	REQUIRE(set.vertices.size() == 5);
	CHECK(set.triangles[0].corners == std::array<std::uint32_t, 3>{ 0u, 1u, 2u });
	CHECK(set.triangles[1].corners == std::array<std::uint32_t, 3>{ 2u, 3u, 4u });
	REQUIRE(set.pieces.size() == 1);
	CHECK(set.pieces[0].name == "dress");
	CHECK(set.pieces[0].firstTriangle == 0u);
	CHECK(set.pieces[0].triangleCount == 2u);

	// The margin and the penetration are the shape's, times the mean of the corners' blended margin multipliers.
	CHECK(set.vertices[0].marginMultiplier == Approx(2.0f));
	CHECK(set.vertices[1].marginMultiplier == Approx(3.0f));
	CHECK(set.vertices[2].marginMultiplier == Approx(4.0f));
	CHECK(set.triangles[0].margin == Approx(kDressMargin * 3.0f));
	CHECK(set.triangles[0].penetration == Approx(kDressPenetration * 3.0f));
	CHECK(set.triangles[1].margin == Approx(kDressMargin * (4.0f + 1.0f + 1.3f) / 3.0f));
	CHECK(set.triangles[0].surfaceMask == kAllMaskBits);

	// A vertex is a blend over its rig bones, each holding it in its own frame.
	const ClothVertex& shared = set.vertices[1];
	CHECK(shared.bone == kLeft);
	CHECK(shared.weight == Approx(0.5f));
	CHECK(shared.local.x == Approx(11.0f));
	CHECK(shared.others[0].bone == kRight);
	CHECK(shared.others[0].weight == Approx(0.5f));
	CHECK(shared.others[0].local.y == Approx(20.0f));
	CHECK(shared.others[1].bone == -1);
	const ClothVertex& mostlyBody = set.vertices[4];
	CHECK(mostlyBody.bone == kBody);
	CHECK(mostlyBody.weight == Approx(0.9f));
	CHECK(mostlyBody.others[0].bone == kRight);

	// A bone owns a triangle once per corner weight above its threshold; flexible is the triangle's mean simulated weight.
	REQUIRE(set.boneTriangles.size() == kRigBones);
	CHECK(set.boneTriangles[kBody].empty());
	REQUIRE(set.boneTriangles[kLeft].size() == 1);
	CHECK(set.boneTriangles[kLeft][0].triangle == 0u);
	CHECK(set.boneTriangles[kLeft][0].corners == 2u);
	CHECK(set.boneTriangles[kLeft][0].flexible == Approx(1.0f));
	REQUIRE(set.boneTriangles[kRight].size() == 2);
	CHECK(set.boneTriangles[kRight][0].triangle == 0u);
	CHECK(set.boneTriangles[kRight][0].corners == 2u);
	CHECK(set.boneTriangles[kRight][1].triangle == 1u);
	CHECK(set.boneTriangles[kRight][1].corners == 2u);
	CHECK(set.boneTriangles[kRight][1].flexible == Approx((1.0f + 0.0f + 0.1f) / 3.0f));
}

TEST_CASE("A corner weight at or below the bone's threshold does not own the triangle", "[smpphysics][clothtriangles]")
{
	Mesh mesh = MakeMesh();
	mesh.weightThreshold[kRight] = 0.5f;
	ClothTriangleSet set;
	REQUIRE(AppendClothTriangles(set, mesh.View("dress")) == 2);
	REQUIRE(set.boneTriangles[kRight].size() == 2);
	CHECK(set.boneTriangles[kRight][0].corners == 1u);
	CHECK(set.boneTriangles[kRight][1].corners == 1u);

	mesh.weightThreshold[kRight] = 1.0f;
	ClothTriangleSet none;
	REQUIRE(AppendClothTriangles(none, mesh.View("dress")) == 1);
	CHECK(none.boneTriangles[kRight].empty());
	CHECK(none.vertices.size() == 3);
}

TEST_CASE("A second shape's triangles follow the first's and are named as their own piece", "[smpphysics][clothtriangles]")
{
	const Mesh mesh = MakeMesh();
	ClothTriangleSet set;
	REQUIRE(AppendClothTriangles(set, mesh.View("dress")) == 2);
	REQUIRE(AppendClothTriangles(set, mesh.View("veil")) == 2);
	REQUIRE(set.pieces.size() == 2);
	CHECK(set.pieces[1].firstTriangle == 2u);
	CHECK(set.triangles[2].corners == std::array<std::uint32_t, 3>{ 5u, 6u, 7u });
	REQUIRE(set.boneTriangles[kLeft].size() == 2);
	CHECK(set.boneTriangles[kLeft][1].triangle == 2u);

	MaskClothPiece(set, set.pieces[1], 0x5u);
	CHECK(set.triangles[1].surfaceMask == kAllMaskBits);
	CHECK(set.triangles[2].surfaceMask == 0x5u);
	CHECK(set.triangles[3].surfaceMask == 0x5u);
}

TEST_CASE("A cloth vertex is posed at the weight blend of its bones", "[smpphysics][clothtriangles]")
{
	const Mesh mesh = MakeMesh();
	ClothTriangleSet set;
	REQUIRE(AppendClothTriangles(set, mesh.View("dress")) == 2);

	// In the bind pose every bone puts the vertex back where the mesh has it.
	std::vector<Transform> bind(kRigBones);
	bind[kLeft] = mesh.skinToBone[0].Inverse();
	bind[kRight] = mesh.skinToBone[1].Inverse();
	bind[kBody] = mesh.skinToBone[2].Inverse();
	for (std::size_t vertex = 0; vertex < set.vertices.size(); ++vertex) {
		const Vec3 posed = PoseClothVertex(set.vertices[vertex], bind);
		CHECK(posed.x == Approx(mesh.vertices[vertex].position.x).margin(1.0e-5f));
		CHECK(posed.y == Approx(mesh.vertices[vertex].position.y).margin(1.0e-5f));
		CHECK(posed.z == Approx(mesh.vertices[vertex].position.z).margin(1.0e-5f));
	}

	std::vector<Transform> moved = bind;
	moved[kRight].translation.z += 2.0f;
	CHECK(PoseClothVertex(set.vertices[1], moved).z == Approx(1.0f));
	CHECK(PoseClothVertex(set.vertices[4], moved).z == Approx(0.2f));
}

TEST_CASE("Adding a contact several times weighs it that many times in a merge", "[smpphysics][clothtriangles]")
{
	const Contact shallow{ 0.2f, { 0.0f, 0.0f, 1.0f }, 0 };
	const Contact deep{ 0.6f, { 1.0f, 0.0f, 0.0f }, 1 };
	ContactLanes counted;
	counted.Add(shallow, { 1.0f, 0.0f, 0.0f }, 7u, 3.0f);
	counted.Add(deep, { 0.0f, 2.0f, 0.0f }, 7u);
	ContactLanes repeated;
	for (int turn = 0; turn < 3; ++turn)
		repeated.Add(shallow, { 1.0f, 0.0f, 0.0f }, 7u);
	repeated.Add(deep, { 0.0f, 2.0f, 0.0f }, 7u);

	Vec3 countedNormal{};
	Vec3 countedPosition{};
	Vec3 repeatedNormal{};
	Vec3 repeatedPosition{};
	CHECK(counted.lanes[0].Result(countedNormal, countedPosition) == Approx(repeated.lanes[0].Result(repeatedNormal, repeatedPosition)));
	CHECK(countedNormal.z == Approx(repeatedNormal.z));
	CHECK(countedPosition.x == Approx(repeatedPosition.x));
	CHECK(counted.lanes[0].weight == Approx(3.0f * 0.04f + 0.36f));
}

TEST_CASE("Cloth triangles pack next to a claim's spheres and move to its slots", "[smpphysics][clothtriangles]")
{
	const Mesh mesh = MakeMesh();
	ClothTriangleSet set;
	REQUIRE(AppendClothTriangles(set, mesh.View("dress")) == 2);
	set.triangles[1].surfaceMask = 0x6u;

	GpuRig::RigData rig;
	PackClothTriangles(rig.collision, set);
	const GpuRig::CollisionData& packed = rig.collision;
	REQUIRE(packed.clothVertices.size() == 5);
	REQUIRE(packed.clothTriangles.size() == 2);
	REQUIRE(packed.boneCloth.size() == kRigBones);
	REQUIRE(packed.clothBoneList.size() == 3);
	CHECK(packed.clothVertices[1].centreRadius.x == Approx(11.0f));
	CHECK(packed.clothVertices[1].exempt.y == static_cast<std::uint32_t>(kLeft));
	CHECK(packed.clothVertices[1].weights.x == Approx(0.5f));
	CHECK(packed.clothVertices[1].otherBone.x == static_cast<std::uint32_t>(kRight));
	CHECK(packed.clothVertices[1].other[0].w == Approx(0.5f));
	CHECK(packed.clothVertices[1].otherBone.y == GpuRig::kNoSphereBone);
	// A triangle names a simulated bone that owns it, whose tree says whether the claim takes contacts.
	CHECK(packed.clothTriangles[0].corners.w == static_cast<std::uint32_t>(kLeft));
	CHECK(packed.clothTriangles[1].corners.w == static_cast<std::uint32_t>(kRight));
	CHECK(packed.clothTriangles[1].contact.x == Approx(set.triangles[1].margin));
	CHECK(packed.clothTriangles[1].contact.y == Approx(set.triangles[1].penetration));
	CHECK(std::bit_cast<std::uint32_t>(packed.clothTriangles[1].contact.z) == 0x6u);
	CHECK(packed.boneCloth[kBody].range.y == 0u);
	CHECK(packed.boneCloth[kLeft].range.x == 0u);
	CHECK(packed.boneCloth[kLeft].range.y == 1u);
	CHECK(packed.boneCloth[kRight].range.x == 1u);
	CHECK(packed.boneCloth[kRight].range.y == 2u);
	CHECK(packed.clothBoneList[2].triangle == 1u);
	CHECK(packed.clothBoneList[2].corners == 2u);
	CHECK(packed.clothBoneList[2].flexible == Approx(1.1f / 3.0f));

	constexpr std::uint32_t kBoneOffset = 64;
	const GpuRig::ClothBase base{ 100u, 200u, 300u };
	const GpuRig::ClaimStatics statics = GpuRig::PackClaimStatics(rig, kBoneOffset, kRigBones, 0u, base);
	REQUIRE(statics.clothVertices.size() == 5);
	CHECK(statics.clothVertices[1].exempt.y == kBoneOffset + kLeft);
	CHECK(statics.clothVertices[1].otherBone.x == kBoneOffset + kRight);
	CHECK(statics.clothVertices[1].otherBone.y == GpuRig::kNoSphereBone);
	CHECK(statics.clothTriangles[1].corners.x == base.vertex + 2u);
	CHECK(statics.clothTriangles[1].corners.w == kBoneOffset + kRight);
	REQUIRE(statics.boneCloth.size() == kRigBones);
	CHECK(statics.boneCloth[kRight].range.x == base.boneList + 1u);
	CHECK(statics.boneCloth[kRight].range.y == 2u);
	CHECK(statics.clothBoneList[2].triangle == base.triangle + 1u);

	// A claim without cloth triangles still packs an empty range for each of its bones.
	const GpuRig::ClaimStatics plain = GpuRig::PackClaimStatics(GpuRig::RigData{}, kBoneOffset, kRigBones, 0u, base);
	REQUIRE(plain.boneCloth.size() == kRigBones);
	CHECK(plain.boneCloth[kRight].range.y == 0u);
	CHECK(plain.clothTriangles.empty());
}

TEST_CASE("A sphere that leaves per-vertex surfaces to its triangles and such a surface are both marked for the shader", "[smpphysics][clothtriangles]")
{
	std::vector<std::vector<BoneSphere>> spheres(1, std::vector<BoneSphere>(2));
	spheres[0][1].triangleContacts = true;
	const GpuRig::CollisionData packed = GpuRig::PackCollision(spheres, {}, 1);
	REQUIRE(packed.sphereList.size() == 2);
	CHECK(packed.sphereList[0].exempt.z == 0u);
	CHECK(packed.sphereList[1].exempt.z == 1u);

	SkinnedSurface surface;
	for (const Vec3& corner : { kA, kB, kC }) {
		SkinnedVertex vertex;
		vertex.position = corner;
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
		surface.vertices.push_back(vertex);
	}
	surface.indices = { 0, 1, 2 };
	surface.skinToBone = { Transform{} };
	surface.boneDriver = { 0 };
	std::vector<SkinnedSurface> surfaces{ surface, surface };
	surfaces[1].perVertex = true;

	GpuRig::SurfaceFrame frame;
	std::uint32_t first = 0;
	std::uint32_t count = 0;
	GpuRig::AppendSurfaces(frame, surfaces, { Transform{} }, first, count);
	REQUIRE(count == 2);
	CHECK(frame.metas[0].base.w == 0u);
	CHECK(frame.metas[1].base.w == 1u);

	GpuRig::SurfacePrepData prep;
	for (const SkinnedSurface& each : surfaces) {
		TriangleBvh bvh;
		bvh.Build(PoseSurface(each, { Transform{} }), each.indices);
		REQUIRE(GpuRig::AppendSurfacePrep(prep, each, bvh, 0u, 1u));
	}
	REQUIRE(prep.metas.size() == 2);
	CHECK(prep.metas[0].base.w == 0u);
	CHECK(prep.metas[1].base.w == 1u);
}
