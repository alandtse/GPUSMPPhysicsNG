// Unit tests for the physics-file problem log (src/smp/RepairLog.cpp). Pure; no game dependency.

#include "smp/RepairLog.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::RepairLog;

TEST_CASE("A problem noted twice counts once and files match without case", "[smpphysics][repairlog]")
{
	RepairLog log;
	log.Record("Meshes/A/Dress.xml", "x is empty", "read as 0");
	log.Record("meshes/a/dress.xml", "x is empty", "read as 0");
	log.Record("meshes/a/dress.xml", "z is empty", "read as 0");
	log.Record("meshes/b.xml", "bad parent", "accepted");

	const auto reports = log.Take();
	REQUIRE(reports.size() == 2);
	CHECK(reports[0].file == "Meshes/A/Dress.xml");
	CHECK(reports[0].entries.size() == 2);
	CHECK(reports[1].entries.size() == 1);
}

TEST_CASE("Empty names and problems are ignored and storage is bounded", "[smpphysics][repairlog]")
{
	RepairLog log;
	log.Record("", "x", "y");
	log.Record("a.xml", "", "y");
	CHECK(log.Take().empty());

	for (std::size_t index = 0; index < RepairLog::kMaxEntriesPerFile + 5; ++index)
		log.Record("a.xml", "problem " + std::to_string(index), "fixed");
	CHECK(log.Take()[0].entries.size() == RepairLog::kMaxEntriesPerFile);

	for (std::size_t index = 0; index < RepairLog::kMaxFiles + 5; ++index)
		log.Record("file" + std::to_string(index) + ".xml", "p", "a");
	CHECK(log.Take().size() == RepairLog::kMaxFiles);

	log.Reset();
	CHECK(log.Take().empty());
}

TEST_CASE("A report reads as text for the file's author", "[smpphysics][repairlog]")
{
	RepairLog log;
	log.Record("meshes/a/dress.xml", "linearUpperLimit x is empty", "read it as 0");
	const std::string text = RepairLog::Format(log.Take()[0]);
	CHECK(text.find("meshes/a/dress.xml") != std::string::npos);
	CHECK(text.find("1. linearUpperLimit x is empty") != std::string::npos);
	CHECK(text.find("-> read it as 0") != std::string::npos);
	CHECK(text.find("What GPU SMP Physics did") == std::string::npos);
}

TEST_CASE("A problem with no action has no action line", "[smpphysics][repairlog]")
{
	RepairLog log;
	log.Record("a.xml", "x is empty and keeps its default", {});
	const std::string text = RepairLog::Format(log.Take()[0]);
	CHECK(text.find("1. x is empty and keeps its default") != std::string::npos);
	CHECK(text.find("->") == std::string::npos);
}

TEST_CASE("Settings and machine declines are not the file's fault", "[smpphysics][repairlog]")
{
	CHECK_FALSE(RepairLog::IsFileFault("collision is switched off in the settings"));
	CHECK_FALSE(RepairLog::IsFileFault("the GPU solver is not available: no device"));
	CHECK_FALSE(RepairLog::IsFileFault("none of the hair meshes 'collision body' is below the actor"));
	CHECK(RepairLog::IsFileFault("bone 'X' has no node below the skeleton"));
}

TEST_CASE("Solver limits and a scene without the mesh are not the file's fault", "[smpphysics][repairlog]")
{
	CHECK_FALSE(RepairLog::IsFileFault("the rigid claim needs more constraint rows or colours than the solver holds"));
	CHECK_FALSE(RepairLog::IsFileFault("the rig needs 900 solver slots, past the 512 one claim may take"));
	CHECK_FALSE(RepairLog::IsFileFault("a chain has more bones than the compute solver serves"));
	CHECK_FALSE(RepairLog::IsFileFault("none of the hair meshes 'MaleHeadCollision' is below the actor"));
	CHECK_FALSE(RepairLog::IsFileFault("the mesh has no skeleton node"));
	CHECK(RepairLog::IsFileFault("no hair spheres could be built for 'Head' (3 of 12 skin bones matched a rig bone)"));
}
