// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/XmlAudit.h"

#include "Utils/StringUtils.h"

#include <algorithm>
#include <map>

namespace SmpPhysics
{
	std::string NormalizeXmlPath(std::string_view a_path)
	{
		std::string path = Util::SearchKey(a_path);
		constexpr std::string_view kData = "data\\";
		if (path.starts_with(kData))
			path.erase(0, kData.size());
		return path;
	}

	bool XmlPathMatches(std::string_view a_path, std::string_view a_needle)
	{
		const std::string needle = Util::SearchKey(a_needle);
		return !needle.empty() && NormalizeXmlPath(a_path).find(needle) != std::string::npos;
	}

	std::vector<std::string> ParseDefaultBbpFiles(std::string_view a_xml)
	{
		std::vector<std::string> files;
		std::size_t cursor = 0;
		while ((cursor = a_xml.find("<map", cursor)) != std::string_view::npos) {
			cursor += 4;
			if (cursor >= a_xml.size() || (a_xml[cursor] != ' ' && a_xml[cursor] != '\t' && a_xml[cursor] != '\n' && a_xml[cursor] != '\r'))
				continue;
			const std::size_t end = a_xml.find('>', cursor);
			if (end == std::string_view::npos)
				break;
			const std::string_view tag = a_xml.substr(cursor, end - cursor);
			std::size_t attribute = 0;
			while ((attribute = tag.find("file", attribute)) != std::string_view::npos) {
				const bool wordStart = attribute == 0 || tag[attribute - 1] == ' ' || tag[attribute - 1] == '\t' || tag[attribute - 1] == '\n' || tag[attribute - 1] == '\r';
				std::size_t value = attribute + 4;
				while (value < tag.size() && tag[value] == ' ')
					++value;
				attribute += 4;
				if (!wordStart || value >= tag.size() || tag[value] != '=')
					continue;
				++value;
				while (value < tag.size() && tag[value] == ' ')
					++value;
				if (value >= tag.size() || (tag[value] != '"' && tag[value] != '\''))
					continue;
				const std::size_t close = tag.find(tag[value], value + 1);
				if (close == std::string_view::npos)
					break;
				files.emplace_back(tag.substr(value + 1, close - value - 1));
				break;
			}
			cursor = end;
		}
		return files;
	}

	XmlAudit AuditXmlFiles(const std::vector<std::string>& a_onDisk, const std::vector<std::string>& a_referenced)
	{
		std::map<std::string, std::string> disk;
		for (const std::string& path : a_onDisk)
			disk.emplace(NormalizeXmlPath(path), path);
		std::map<std::string, std::string> named;
		for (const std::string& path : a_referenced)
			named.emplace(NormalizeXmlPath(path), path);

		XmlAudit audit;
		for (const auto& [key, original] : disk) {
			if (!named.contains(key))
				audit.orphans.push_back(original);
		}
		for (const auto& [key, original] : named) {
			if (!disk.contains(key))
				audit.dangling.push_back(original);
		}
		return audit;
	}
}
