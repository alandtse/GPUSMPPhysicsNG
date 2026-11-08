#include "Utils/DevBenchUx.h"

#include <set>

namespace Util::DevBenchUx
{
	Registry& Registry::GetSingleton()
	{
		static Registry singleton;
		return singleton;
	}

	void Registry::RegisterCommand(std::string_view a_featureShortName, std::string_view a_name, std::string_view a_description, CommandFn a_fn, ExecutionThread a_thread)
	{
		commands[std::string(a_featureShortName)][std::string(a_name)] = { std::string(a_description), a_fn, a_thread };
	}

	void Registry::RegisterQuery(std::string_view a_featureShortName, std::string_view a_name, std::string_view a_description, QueryFn a_fn, ExecutionThread a_thread)
	{
		queries[std::string(a_featureShortName)][std::string(a_name)] = { std::string(a_description), a_fn, a_thread };
	}

	void Registry::RegisterToolCommand(std::string_view a_owner, std::string_view a_name, std::string_view a_description, ToolCommandFn a_fn)
	{
		toolCommands[std::string(a_owner)][std::string(a_name)] = { std::string(a_description), a_fn };
	}

	void Registry::RegisterToolQuery(std::string_view a_owner, std::string_view a_name, std::string_view a_description, ToolQueryFn a_fn)
	{
		toolQueries[std::string(a_owner)][std::string(a_name)] = { std::string(a_description), a_fn };
	}

	const CommandEntry* Registry::FindCommand(std::string_view a_featureShortName, std::string_view a_name) const
	{
		const auto featureIt = commands.find(std::string(a_featureShortName));
		if (featureIt == commands.end())
			return nullptr;
		const auto entryIt = featureIt->second.find(std::string(a_name));
		return entryIt == featureIt->second.end() ? nullptr : &entryIt->second;
	}

	const QueryEntry* Registry::FindQuery(std::string_view a_featureShortName, std::string_view a_name) const
	{
		const auto featureIt = queries.find(std::string(a_featureShortName));
		if (featureIt == queries.end())
			return nullptr;
		const auto entryIt = featureIt->second.find(std::string(a_name));
		return entryIt == featureIt->second.end() ? nullptr : &entryIt->second;
	}

	const ToolCommandEntry* Registry::FindToolCommand(std::string_view a_owner, std::string_view a_name) const
	{
		const auto ownerIt = toolCommands.find(std::string(a_owner));
		if (ownerIt == toolCommands.end())
			return nullptr;
		const auto entryIt = ownerIt->second.find(std::string(a_name));
		return entryIt == ownerIt->second.end() ? nullptr : &entryIt->second;
	}

	const ToolQueryEntry* Registry::FindToolQuery(std::string_view a_owner, std::string_view a_name) const
	{
		const auto ownerIt = toolQueries.find(std::string(a_owner));
		if (ownerIt == toolQueries.end())
			return nullptr;
		const auto entryIt = ownerIt->second.find(std::string(a_name));
		return entryIt == ownerIt->second.end() ? nullptr : &entryIt->second;
	}

	bool Registry::HasToolOwner(std::string_view a_owner) const
	{
		return toolCommands.contains(std::string(a_owner)) || toolQueries.contains(std::string(a_owner));
	}

	json Registry::ListCommands(std::string_view a_featureShortName) const
	{
		json out = json::array();
		const auto featureIt = commands.find(std::string(a_featureShortName));
		if (featureIt != commands.end())
			for (const auto& [name, entry] : featureIt->second)
				out.push_back(json{ { "name", name }, { "description", entry.description }, { "executionThread", entry.executionThread == ExecutionThread::Render ? "render" : "main" } });
		return out;
	}

	json Registry::ListQueries(std::string_view a_featureShortName) const
	{
		json out = json::array();
		const auto featureIt = queries.find(std::string(a_featureShortName));
		if (featureIt != queries.end())
			for (const auto& [name, entry] : featureIt->second)
				out.push_back(json{ { "name", name }, { "description", entry.description }, { "executionThread", entry.executionThread == ExecutionThread::Render ? "render" : "main" } });
		return out;
	}

	json Registry::ListToolCommands(std::string_view a_owner) const
	{
		json out = json::array();
		const auto ownerIt = toolCommands.find(std::string(a_owner));
		if (ownerIt != toolCommands.end())
			for (const auto& [name, entry] : ownerIt->second)
				out.push_back(json{ { "name", name }, { "description", entry.description } });
		return out;
	}

	json Registry::ListToolQueries(std::string_view a_owner) const
	{
		json out = json::array();
		const auto ownerIt = toolQueries.find(std::string(a_owner));
		if (ownerIt != toolQueries.end())
			for (const auto& [name, entry] : ownerIt->second)
				out.push_back(json{ { "name", name }, { "description", entry.description } });
		return out;
	}

	json Registry::ListToolOwners() const
	{
		std::set<std::string> owners;
		for (const auto& entry : toolCommands)
			owners.insert(entry.first);
		for (const auto& entry : toolQueries)
			owners.insert(entry.first);
		json out = json::array();
		for (const std::string& owner : owners)
			out.push_back(owner);
		return out;
	}
}
