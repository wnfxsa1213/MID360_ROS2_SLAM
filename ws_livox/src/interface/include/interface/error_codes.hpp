#pragma once

#include <cstdint>
#include <string>

namespace interface::errors {

enum class Code : std::int32_t
{
    SUCCESS = 0,
    INVALID_CONFIGURATION = 1001,
    FILE_NOT_FOUND = 1002,
    DATA_MISMATCH = 1003,
    SERVICE_UNAVAILABLE = 2001,
    TIMEOUT = 2002,
    OPTIMIZATION_FAILED = 3001,
    VALIDATION_FAILED = 4001,
    INTERNAL_ERROR = 9001
};

inline const char* toString(Code code)
{
    switch (code)
    {
    case Code::SUCCESS: return "SUCCESS";
    case Code::INVALID_CONFIGURATION: return "INVALID_CONFIGURATION";
    case Code::FILE_NOT_FOUND: return "FILE_NOT_FOUND";
    case Code::DATA_MISMATCH: return "DATA_MISMATCH";
    case Code::SERVICE_UNAVAILABLE: return "SERVICE_UNAVAILABLE";
    case Code::TIMEOUT: return "TIMEOUT";
    case Code::OPTIMIZATION_FAILED: return "OPTIMIZATION_FAILED";
    case Code::VALIDATION_FAILED: return "VALIDATION_FAILED";
    case Code::INTERNAL_ERROR: return "INTERNAL_ERROR";
    default: return "UNKNOWN";
    }
}

inline std::string format(Code code, const std::string& detail)
{
    const auto numeric = static_cast<std::int32_t>(code);
    std::string formatted = "E" + std::to_string(numeric) + ":" + toString(code);
    if (!detail.empty())
    {
        formatted.append(" - ").append(detail);
    }
    return formatted;
}

} // namespace interface::errors

