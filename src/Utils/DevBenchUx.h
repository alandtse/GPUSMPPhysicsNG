#pragma once

#include <string>
#include <string_view>
#include <unordered_map>

struct Feature;

namespace Util::DevBenchUx
{
	using CommandFn = void (*)(Feature*, const json&);
	using QueryFn = json (*)(const Feature*, const json&);
	using ToolCommandFn = void (*)(const json&);
	using ToolQueryFn = json (*)(const json&);

	/** @brief The thread that owns a registered action's live state. */
	enum class ExecutionThread
	{
		Main,
		Render
	};

	/** @brief A registered command: its devbench-facing description and callback. */
	struct CommandEntry
	{
		std::string description;
		CommandFn fn;
		ExecutionThread executionThread = ExecutionThread::Main;
	};

	/** @brief A registered query: its devbench-facing description and callback. */
	struct QueryEntry
	{
		std::string description;
		QueryFn fn;
		ExecutionThread executionThread = ExecutionThread::Main;
	};

	/** @brief A registered non-feature tool command: its devbench-facing description and callback. */
	struct ToolCommandEntry
	{
		std::string description;
		ToolCommandFn fn;
	};

	/** @brief A registered non-feature tool query: its devbench-facing description and callback. */
	struct ToolQueryEntry
	{
		std::string description;
		ToolQueryFn fn;
	};

	/**
	 * @brief Per-feature tables of devbench-invocable commands (one-shot, non-settings
	 * actions) and queries (read-only computed values), the imperative/derived-state
	 * counterpart to how Feature::SaveSettings/LoadSettings already make settings fields
	 * devbench-drivable with no bridge changes. Holds commands of non-feature tool owners
	 * the same way. Populated once at boot by DevBenchBridge::Install() from each
	 * Feature::RegisterUxActions() override -- see FEATURE_COMMAND/FEATURE_QUERY below.
	 * Immutable after boot: safe for the devbench listener thread to read without locking.
	 */
	class Registry
	{
	public:
		/** @brief Process-wide instance; there is exactly one registry. */
		static Registry& GetSingleton();

		/** @brief Registers a command under a feature's shortName. Use FEATURE_COMMAND, not this directly. */
		void RegisterCommand(std::string_view a_featureShortName, std::string_view a_name, std::string_view a_description, CommandFn a_fn, ExecutionThread a_thread = ExecutionThread::Main);
		/** @brief Registers a query under a feature's shortName. Use FEATURE_QUERY, not this directly. */
		void RegisterQuery(std::string_view a_featureShortName, std::string_view a_name, std::string_view a_description, QueryFn a_fn, ExecutionThread a_thread = ExecutionThread::Main);

		/** @brief Registers a command under a tool owner that is not a feature. Use TOOL_COMMAND, not this directly. */
		void RegisterToolCommand(std::string_view a_owner, std::string_view a_name, std::string_view a_description, ToolCommandFn a_fn);
		/** @brief Registers a query under a tool owner that is not a feature. Use TOOL_QUERY, not this directly. */
		void RegisterToolQuery(std::string_view a_owner, std::string_view a_name, std::string_view a_description, ToolQueryFn a_fn);

		/** @return The matching command, or nullptr if no feature/name combination is registered. */
		const CommandEntry* FindCommand(std::string_view a_featureShortName, std::string_view a_name) const;
		/** @return The matching query, or nullptr if no feature/name combination is registered. */
		const QueryEntry* FindQuery(std::string_view a_featureShortName, std::string_view a_name) const;

		/** @return The matching tool command, or nullptr if no owner/name combination is registered. */
		const ToolCommandEntry* FindToolCommand(std::string_view a_owner, std::string_view a_name) const;
		/** @return The matching tool query, or nullptr if no owner/name combination is registered. */
		const ToolQueryEntry* FindToolQuery(std::string_view a_owner, std::string_view a_name) const;

		/** @return Whether this owner has any tool command or query. */
		bool HasToolOwner(std::string_view a_owner) const;

		/** @return `[{name, description, executionThread}, ...]` for this feature's commands. */
		json ListCommands(std::string_view a_featureShortName) const;
		/** @return `[{name, description, executionThread}, ...]` for this feature's queries. */
		json ListQueries(std::string_view a_featureShortName) const;

		/** @return `[{name, description}, ...]` for every command registered under this tool owner. */
		json ListToolCommands(std::string_view a_owner) const;
		/** @return `[{name, description}, ...]` for every query registered under this tool owner. */
		json ListToolQueries(std::string_view a_owner) const;
		/** @return A sorted array of every tool owner's name, without duplicates. */
		json ListToolOwners() const;

	private:
		std::unordered_map<std::string, std::unordered_map<std::string, CommandEntry>> commands;
		std::unordered_map<std::string, std::unordered_map<std::string, QueryEntry>> queries;
		std::unordered_map<std::string, std::unordered_map<std::string, ToolCommandEntry>> toolCommands;
		std::unordered_map<std::string, std::unordered_map<std::string, ToolQueryEntry>> toolQueries;
	};
}

/**
 * @brief Registers a devbench command from a Feature::RegisterUxActions override.
 * @param Fn Capture-less lambda `[](Feature* self, const json& args) { ... }` (the unary
 * `+` casts it to a plain function pointer -- a capturing lambda won't compile here).
 */
#define FEATURE_COMMAND(Name, Desc, Fn) \
	::Util::DevBenchUx::Registry::GetSingleton().RegisterCommand(GetShortName(), Name, Desc, +Fn)

/** @brief Same contract as FEATURE_COMMAND, for a read-only computed value.
 * @param Fn Capture-less lambda `[](const Feature* self, const json& args) -> json { ... }`. */
#define FEATURE_QUERY(Name, Desc, Fn) \
	::Util::DevBenchUx::Registry::GetSingleton().RegisterQuery(GetShortName(), Name, Desc, +Fn)

/**
 * @brief Registers a devbench command under a tool owner that is not a Feature.
 * @param Owner A short name that must not equal any Feature::GetShortName(); the owner is refused otherwise.
 * @param Fn Capture-less lambda `[](const json& args) { ... }`.
 */
#define TOOL_COMMAND(Owner, Name, Desc, Fn) \
	::Util::DevBenchUx::Registry::GetSingleton().RegisterToolCommand(Owner, Name, Desc, +Fn)

/** @brief Same contract as TOOL_COMMAND, for a read-only computed value.
 * @param Fn Capture-less lambda `[](const json& args) -> json { ... }`. */
#define TOOL_QUERY(Owner, Name, Desc, Fn) \
	::Util::DevBenchUx::Registry::GetSingleton().RegisterToolQuery(Owner, Name, Desc, +Fn)
