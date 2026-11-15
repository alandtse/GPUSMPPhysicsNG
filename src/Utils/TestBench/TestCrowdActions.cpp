// Devbench commands and queries that spawn, dress, turn and animate test characters in front of the player.
// The crowd is feature-neutral; features add their own touches through TestBench::CrowdHooks.

#include "Utils/TestBench/TestCrowdActions.h"

#include "Utils/DevBenchUx.h"
#include "Utils/StringUtils.h"
#include "Utils/TestBench/CrowdPool.h"
#include "Utils/TestBench/IdleCache.h"
#include "Utils/TestBench/TestCrowd.h"
#include "Utils/TestBench/TestCrowdLayout.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <random>
#include <string>

using json = nlohmann::json;

namespace
{
	json TestCrowdJson(const TestBench::TestCrowd& a_crowd)
	{
		json result;
		result["count"] = a_crowd.Count();
		result["report"] = a_crowd.Report();
		return result;
	}
}

void TestBench::RegisterUxActions()
{
	TOOL_COMMAND(kOwnerName, "spawnTestCrowd", "Spawn characters in rows ahead of the player, standing still and dressed by whatever features hook into the crowd (GPU SMP Physics gives each a wig), reusing the characters of an earlier spawn. Args: count (number, 1 to 60, default 10), bases (optional array of NPC base form IDs, as numbers or 0x strings, taken in turn so the crowd mixes characters; default one generic female character; a character left from an earlier spawn is reused only when its base matches), anyAdultWoman (bool, default false; picks from every named female NPC of a humanoid race the game loaded that is friendly or neutral to the player (never a bandit or other enemy) instead of bases), randomOrder (bool, default false; shuffles the picks), excludeChildren and excludeElderly (bool; leave out child races and elderly races, default true with anyAdultWoman and false otherwise). The pick and what the filter left out are logged as [Test Crowd] Character pool. Read what happened with testCrowdReport.",
		[](const json& a_args) {
			TestBench::SpawnChoice choice;
			choice.count = static_cast<std::size_t>(std::clamp(a_args.value("count", 10), 1, static_cast<int>(TestBench::kMaxTestCrowd)));
			choice.anyAdultWoman = a_args.value("anyAdultWoman", false);
			choice.randomOrder = a_args.value("randomOrder", false);
			choice.seed = std::random_device{}();
			choice.filter.excludeChildren = a_args.value("excludeChildren", choice.anyAdultWoman);
			choice.filter.excludeElderly = a_args.value("excludeElderly", choice.anyAdultWoman);
			if (a_args.contains("bases") && a_args["bases"].is_array()) {
				for (const json& base : a_args["bases"]) {
					if (base.is_number_unsigned())
						choice.candidates.push_back(base.get<std::uint32_t>());
					else if (base.is_string()) {
						try {
							choice.candidates.push_back(static_cast<std::uint32_t>(std::stoul(base.get<std::string>(), nullptr, 0)));
						} catch (const std::exception&) {
							continue;
						}
					}
				}
			}
			if (!choice.anyAdultWoman && choice.candidates.empty())
				TestBench::GetCrowd().Spawn(choice.count, {});
			else
				TestBench::SpawnCrowd(std::move(choice));
		});
	TOOL_COMMAND(kOwnerName, "refreshTestCrowd", "Rebuild the 3D of every test character standing now, then let the feature hooks redo what a rebuild undoes (GPU SMP Physics hides baked hair under wigs again); use it after changing a base record, for example its hair colour. Read the outcome from testCrowdReport.",
		[](const json&) { TestBench::GetCrowd().Refresh(); });
	TOOL_COMMAND(kOwnerName, "turnTestCrowd", "Turn test characters to face the player rotated by some degrees, so a camera that stays put shows them from the front (0), the side (90 or 270) or the back (180). Args: index (number, which character in spawn order, default all), degrees (number, default 0). Read the outcome from testCrowdReport.",
		[](const json& a_args) { TestBench::GetCrowd().Turn(a_args.value("index", -1), a_args.value("degrees", 0.0f)); });
	TOOL_COMMAND(kOwnerName, "stripTestCrowd", "Take armor and weapons off the test characters standing now, except what a feature hook keeps (wigs, for GPU SMP Physics), so armor can be compared against bare bodies; later spawns start stripped while it is on. Args: enabled (bool, default true; false puts their outfits back).",
		[](const json& a_args) { TestBench::GetCrowd().SetStripped(a_args.value("enabled", true)); });
	TOOL_COMMAND(kOwnerName, "setTestCrowdIncludePlayer", "Make stripTestCrowd, cycleTestCrowdArmor and setTestCrowdCloak act on the player as well as the spawned characters, so a worn outfit can be checked on the player's own body. Turning it on dresses the player in what the crowd wears now; off puts the player's own outfit back and takes the cloak off. Args: enabled (bool, default true).",
		[](const json& a_args) { TestBench::GetCrowd().SetIncludePlayer(a_args.value("enabled", true)); });
	TOOL_COMMAND(kOwnerName, "cycleTestCrowdArmor", "Dress every test character in the next or previous playable body armor, with weapons and other gear off and what hooks keep left on, to compare armor types. Args: step (number, default 1; -1 goes back). Read the armor worn from testCrowdReport.",
		[](const json& a_args) { TestBench::GetCrowd().CycleArmor(a_args.value("step", 1)); });
	TOOL_COMMAND(kOwnerName, "setTestCrowdArmor", "Dress every test character in one playable body armor by name, so a run can use a chosen outfit. Args: name (string, case-insensitive part of an armor's name; the first match is worn).",
		[](const json& a_args) {
			const std::string name = Util::ToLowerAscii(a_args.value("name", std::string{}));
			const auto& names = TestBench::TestCrowd::ArmorNames();
			const auto match = std::find_if(names.begin(), names.end(), [&](const std::string& a_name) { return Util::ToLowerAscii(a_name).find(name) != std::string::npos; });
			if (!name.empty() && match != names.end())
				TestBench::GetCrowd().SelectArmor(static_cast<std::size_t>(match - names.begin()));
		});
	TOOL_COMMAND(kOwnerName, "setTestCrowdCloak", "Put a cloak or cape on every test character standing now, and on each one spawned later; it is a layer of its own that stays on through stripping and armor changes. Args: name (string, case-insensitive part of an entry of testCrowdCloaks; the first match is worn), index (number, an entry of testCrowdCloaks), or neither / index -1 to take the cloak off. Read the outcome from testCrowdReport.",
		[](const json& a_args) {
			int index = a_args.value("index", -1);
			const std::string name = Util::ToLowerAscii(a_args.value("name", std::string{}));
			if (!name.empty()) {
				const auto& names = TestBench::TestCrowd::CloakNames();
				const auto match = std::find_if(names.begin(), names.end(), [&](const std::string& a_name) { return Util::ToLowerAscii(a_name).find(name) != std::string::npos; });
				index = match != names.end() ? static_cast<int>(match - names.begin()) : -1;
			}
			TestBench::GetCrowd().SelectCloak(index);
		});
	TOOL_QUERY(kOwnerName, "testCrowdCloaks", "List the cloaks and capes the test crowd can wear, with their index for setTestCrowdCloak, and which one it wears now (worn is -1 for none). Args: filter (string, case-insensitive part of the name, default none), limit (number, default 100).",
		[](const json& a_args) -> json {
			const std::string filter = Util::ToLowerAscii(a_args.value("filter", std::string{}));
			const std::size_t limit = static_cast<std::size_t>(std::max(a_args.value("limit", 100), 1));
			json rows = json::array();
			const auto& names = TestBench::TestCrowd::CloakNames();
			for (std::size_t index = 0; index < names.size() && rows.size() < limit; ++index) {
				if (filter.empty() || Util::ToLowerAscii(names[index]).find(filter) != std::string::npos)
					rows.push_back({ { "index", index }, { "name", names[index] } });
			}
			json result;
			result["worn"] = TestBench::GetCrowd().WornCloakIndex();
			result["cloaks"] = rows;
			return result;
		});
	TOOL_COMMAND(kOwnerName, "playTestCrowdAnimation", "Play an idle animation on every test character standing now, and on each one spawned later, to test a feature on moving characters; the default is a Cicero idle. Args: idle (string, an editor ID from testCrowdAnimations; ResetRoot stops; empty plays the default). Read how many played from testCrowdReport.",
		[](const json& a_args) { TestBench::GetCrowd().PlayAnimation(a_args.value("idle", std::string{})); });
	TOOL_COMMAND(kOwnerName, "setTestCrowdLoop", "Make the test characters play their idle again whenever it ends, so a dance repeats indefinitely, or stop doing so. Args: enabled (bool, default true).",
		[](const json& a_args) { TestBench::GetCrowd().SetLoopAnimation(a_args.value("enabled", true)); });
	TOOL_QUERY(kOwnerName, "testCrowdAnimations", "List the game's idle animations by editor ID with the event that plays each; paired animations are left out. Args: filter (string, case-insensitive part of the editor ID, default none), limit (number, default 100).",
		[](const json& a_args) -> json {
			const std::string filter = Util::ToLowerAscii(a_args.value("filter", std::string{}));
			const std::size_t limit = static_cast<std::size_t>(std::max(a_args.value("limit", 100), 1));
			json rows = json::array();
			for (const TestBench::IdleChoice& idle : TestBench::IdleCache::Snapshot()) {
				if (rows.size() >= limit)
					break;
				if (filter.empty() || Util::ToLowerAscii(idle.editorId).find(filter) != std::string::npos)
					rows.push_back({ { "editorId", idle.editorId }, { "event", idle.event } });
			}
			return rows;
		});
	TOOL_COMMAND(kOwnerName, "clearTestCrowd", "Remove the characters spawnTestCrowd made.",
		[](const json&) { TestBench::GetCrowd().Clear(); });
	TOOL_QUERY(kOwnerName, "testCrowdOutfit", "What each test character wears now: its form ID, worn (the names of worn armor and weapons) and kept (worn items a feature hook keeps on through stripping, such as wigs). Use it to check that stripping left only the kept items and that armor cycling dressed everyone.",
		[](const json&) -> json {
			json rows = json::array();
			for (const auto& summary : TestBench::GetCrowd().Worn())
				rows.push_back({ { "actorFormID", summary.actor }, { "worn", summary.worn }, { "kept", summary.kept } });
			return rows;
		});
	TOOL_QUERY(kOwnerName, "testCrowdReport", "How many test characters stand now and what the last spawn or removal did.",
		[](const json&) -> json { return TestCrowdJson(TestBench::GetCrowd()); });
}
