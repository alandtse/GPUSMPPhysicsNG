// Unit tests for how often a claim's worn meshes are scanned (src/smp/BodyScanPacing.h).
// Pure logic; no game/D3D dependency.

#include "smp/BodyScanPacing.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("A settled actor nothing touched is scanned on the idle interval", "[smpphysics][bodyscan]")
{
	BodyScanWindows windows;
	CHECK(BodyScanInterval(true, windows.Open(0x14, 0.016)) == kBodyScanIdleSeconds);
}

TEST_CASE("A claim still settling is scanned on the active interval", "[smpphysics][bodyscan]")
{
	CHECK(BodyScanInterval(false, false) == kBodyScanActiveSeconds);
}

TEST_CASE("An event opens a window that closes once its time is spent", "[smpphysics][bodyscan]")
{
	BodyScanWindows windows;
	windows.Note(0x14);
	CHECK(BodyScanInterval(true, windows.Open(0x14, 1.0)) == kBodyScanActiveSeconds);
	CHECK(windows.Open(0x14, kBodyScanWindowSeconds - 2.0));
	CHECK_FALSE(windows.Open(0x14, kBodyScanWindowSeconds));
	CHECK(BodyScanInterval(true, windows.Open(0x14, 0.016)) == kBodyScanIdleSeconds);
}

TEST_CASE("An event for one actor leaves the others idle", "[smpphysics][bodyscan]")
{
	BodyScanWindows windows;
	windows.Note(0x14);
	CHECK_FALSE(windows.Open(0x1A67B, 0.016));
}

TEST_CASE("A second event restarts the window", "[smpphysics][bodyscan]")
{
	BodyScanWindows windows;
	windows.Note(0x14);
	CHECK(windows.Open(0x14, kBodyScanWindowSeconds - 1.0));
	windows.Note(0x14);
	CHECK(windows.Open(0x14, kBodyScanWindowSeconds - 1.0));
}
