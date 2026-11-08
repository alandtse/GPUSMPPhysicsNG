#pragma once

#include <format>
#include <functional>
#include <string>

/** @brief Minimal logging facade: the plugin routes the sink to spdlog, tests leave it silent. */
namespace smplog
{
	enum class Level
	{
		Debug,
		Info,
		Warn,
		Error
	};

	using Sink = std::function<void(Level, const std::string&)>;

	inline Sink& GetSink()
	{
		static Sink sink;
		return sink;
	}

	template <class... Args>
	void Log(Level a_level, std::format_string<Args...> a_format, Args&&... a_args)
	{
		if (const auto& sink = GetSink())
			sink(a_level, std::format(a_format, std::forward<Args>(a_args)...));
	}

	template <class... Args>
	void debug(std::format_string<Args...> a_format, Args&&... a_args)
	{
		Log(Level::Debug, a_format, std::forward<Args>(a_args)...);
	}
	template <class... Args>
	void info(std::format_string<Args...> a_format, Args&&... a_args)
	{
		Log(Level::Info, a_format, std::forward<Args>(a_args)...);
	}
	template <class... Args>
	void warn(std::format_string<Args...> a_format, Args&&... a_args)
	{
		Log(Level::Warn, a_format, std::forward<Args>(a_args)...);
	}
	template <class... Args>
	void error(std::format_string<Args...> a_format, Args&&... a_args)
	{
		Log(Level::Error, a_format, std::forward<Args>(a_args)...);
	}
}

#ifndef SMP_USE_SKSE_LOG
namespace logger = smplog;
#endif
