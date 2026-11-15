#include "pch.h"

#include "plugin/DevBench.h"

#include "Utils/DevBenchUx.h"
#include "Utils/TestBench/TestCrowdActions.h"
#include "plugin/Instance.h"

#include <DevBenchAPI.h>

#include <chrono>
#include <functional>
#include <future>

namespace
{
	constexpr const char* kToolName = "gpusmp.actions";
	constexpr auto kQueryTimeout = std::chrono::milliseconds(5000);

	constexpr const char* kToolDescriptor =
		R"({"description":"GPUSMPPhysicsNG commands and read-only queries. Owners: SmpPhysics (the plugin itself: status, solverBones, reevaluate, setClaimEnabled, probes and audits) and TestCrowd (spawnTestCrowd, clearTestCrowd, outfits, animations and hair colours for a crowd of test characters). Actions: listCommands/listQueries (param shortName; with none they list every owner), invokeCommand (queued for the game thread) and invokeQuery (waits up to 5000 ms and returns the result). Unknown owners or names return a plain error.",)"
		R"("inputSchema":{"type":"object","properties":{)"
		R"("action":{"type":"string","enum":["listCommands","listQueries","invokeCommand","invokeQuery"],"description":"List registered actions, queue a command, or wait for a query."},)"
		R"("shortName":{"type":"string","description":"SmpPhysics or TestCrowd; omit to list the owners."},)"
		R"("name":{"type":"string","description":"The registered command or query name."},)"
		R"("args":{"type":"object","description":"Arguments for the command or query; each description lists its own."}},)"
		R"("required":["action"]}})";

	/** @brief Runs a_run on the game thread and returns its result; reports an error when the thread does not get to it in time. */
	json RunOnGameThread(std::function<json()> a_run)
	{
		auto* task = SKSE::GetTaskInterface();
		if (!task)
			return json{ { "error", "SKSE task interface unavailable" } };
		auto promise = std::make_shared<std::promise<json>>();
		auto cancelled = std::make_shared<std::atomic<bool>>(false);
		auto future = promise->get_future();
		task->AddTask([promise, cancelled, run = std::move(a_run)] {
			if (cancelled->load(std::memory_order_acquire))
				return;
			try {
				promise->set_value(run());
			} catch (const std::exception& e) {
				promise->set_value(json{ { "error", "query threw" }, { "detail", e.what() } });
			}
		});
		if (future.wait_for(kQueryTimeout) != std::future_status::ready) {
			cancelled->store(true, std::memory_order_release);
			return json{ { "error", "the game thread did not run the query within " + std::to_string(kQueryTimeout.count()) + " ms (mid-load?)" } };
		}
		return future.get();
	}

	json ListOwners()
	{
		json rows = json::array();
		rows.push_back({ { "shortName", Instance().GetShortName() }, { "kind", "feature" } });
		for (const auto& owner : Util::DevBenchUx::Registry::GetSingleton().ListToolOwners())
			rows.push_back({ { "shortName", owner }, { "kind", "tool" } });
		return json{ { "owners", rows } };
	}

	json BuildResult(const json& a_args)
	{
		auto& registry = Util::DevBenchUx::Registry::GetSingleton();
		auto& feature = Instance();
		const std::string action = a_args.value("action", std::string{});
		const std::string shortName = a_args.value("shortName", std::string{});
		const std::string name = a_args.value("name", std::string{});
		const json actionArgs = a_args.value("args", json::object());
		const bool isFeature = shortName == feature.GetShortName();

		if (action == "listCommands" || action == "listQueries") {
			const bool commands = action == "listCommands";
			const char* key = commands ? "commands" : "queries";
			if (isFeature)
				return json{ { "shortName", shortName }, { key, commands ? registry.ListCommands(shortName) : registry.ListQueries(shortName) } };
			if (registry.HasToolOwner(shortName))
				return json{ { "shortName", shortName }, { key, commands ? registry.ListToolCommands(shortName) : registry.ListToolQueries(shortName) } };
			if (shortName.empty())
				return ListOwners();
			return json{ { "error", "unknown or missing shortName" }, { "shortName", shortName } };
		}

		if (action == "invokeCommand") {
			std::function<void()> run;
			const auto* featureEntry = isFeature ? registry.FindCommand(shortName, name) : nullptr;
			if (featureEntry)
				run = [fn = featureEntry->fn, &feature, actionArgs] { fn(&feature, actionArgs); };
			else if (const auto* toolEntry = registry.FindToolCommand(shortName, name))
				run = [fn = toolEntry->fn, actionArgs] { fn(actionArgs); };
			else
				return json{ { "error", "unknown command" }, { "shortName", shortName }, { "name", name } };
			auto* task = SKSE::GetTaskInterface();
			if (!task)
				return json{ { "error", "SKSE task interface unavailable" } };
			task->AddTask([run = std::move(run), shortName, name] {
				try {
					run();
					logger::info("DevBench: {}.{} ran", shortName, name);
				} catch (const std::exception& e) {
					logger::error("DevBench: {}.{} threw: {}", shortName, name, e.what());
				}
			});
			return json{ { "action", action }, { "shortName", shortName }, { "name", name }, { "queued", true } };
		}

		if (action == "invokeQuery") {
			if (const auto* featureEntry = isFeature ? registry.FindQuery(shortName, name) : nullptr)
				return RunOnGameThread([fn = featureEntry->fn, &feature, actionArgs] { return fn(&feature, actionArgs); });
			if (const auto* toolEntry = registry.FindToolQuery(shortName, name))
				return RunOnGameThread([fn = toolEntry->fn, actionArgs] { return fn(actionArgs); });
			return json{ { "error", "unknown query" }, { "shortName", shortName }, { "name", name } };
		}

		return json{ { "error", "unknown action (listCommands|listQueries|invokeCommand|invokeQuery)" }, { "action", action } };
	}

	/** @brief The C-ABI handler: no exception crosses into devbench, and the result is written exactly once. */
	void ActionsHandler(void*, const char* a_argsJson, void* a_sink, DevBenchAPI::WriteFn a_write)
	{
		json out;
		try {
			json args = a_argsJson && *a_argsJson ? json::parse(a_argsJson) : json::object();
			out = args.is_object() ? BuildResult(args) : json{ { "error", "arguments must be a JSON object" } };
		} catch (const std::exception& e) {
			out = json{ { "error", "invalid request" }, { "detail", e.what() } };
		}
		const std::string dumped = out.dump(-1, ' ', false, json::error_handler_t::replace);
		a_write(a_sink, dumped.c_str());
	}
}

void DevBench::Install()
{
	Instance().RegisterUxActions();
	TestBench::RegisterUxActions();
	auto* devbench = DevBenchAPI::GetDevBenchInterface001();
	if (!devbench) {
		logger::info("devbench is not loaded; the {} tool is not registered", kToolName);
		return;
	}
	devbench->RegisterTool(kToolName, kToolDescriptor, &ActionsHandler, nullptr);
	logger::info("Registered the {} devbench tool (devbench build {})", kToolName, devbench->GetBuildNumber());
}
