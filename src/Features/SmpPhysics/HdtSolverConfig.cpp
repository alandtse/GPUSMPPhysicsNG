#include "Features/SmpPhysics/HdtSolverConfig.h"

#include <nlohmann/json.hpp>

#include <algorithm>

namespace SmpPhysics
{
	namespace
	{
		void ApplySolverSection(std::string_view a_text, HdtSolverConfig& a_config)
		{
			constexpr std::string_view kByteOrderMark = "\xEF\xBB\xBF";
			if (a_text.starts_with(kByteOrderMark))
				a_text.remove_prefix(kByteOrderMark.size());
			const nlohmann::json document = nlohmann::json::parse(a_text, nullptr, false, true);
			if (!document.is_object())
				return;
			const auto solver = document.find("solver");
			if (solver == document.end() || !solver->is_object())
				return;
			const auto number = [&solver](const char* a_key, auto a_current) {
				const auto found = solver->find(a_key);
				return found != solver->end() && found->is_number() ? found->get<decltype(a_current)>() : a_current;
			};
			a_config.iterations = std::clamp(number("numIterations", a_config.iterations), 4, 128);
			a_config.erp = std::clamp(number("erp", a_config.erp), 0.01f, 1.0f);
			a_config.minFps = std::clamp(number("min-fps", a_config.minFps), 1, 300);
			a_config.maxSubSteps = std::clamp(number("maxSubSteps", a_config.maxSubSteps), 1, 60);
		}
	}

	HdtSolverConfig ParseHdtSolverConfig(std::string_view a_defaults, std::string_view a_user)
	{
		HdtSolverConfig config;
		ApplySolverSection(a_defaults, config);
		ApplySolverSection(a_user, config);
		return config;
	}
}
