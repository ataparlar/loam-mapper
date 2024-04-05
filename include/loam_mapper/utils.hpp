
#ifndef BUILD_UTILS_HPP
#define BUILD_UTILS_HPP

#include <string>
#include <cstdint>
#include <vector>
#include <cmath>
#include <type_traits>

namespace loam_mapper::utils
{

    class Utils
    {
    public:
        static std::string byte_hex_to_string(uint8_t byte_hex);
        static std::string bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes);
        static std::vector<std::string> string_to_vec_split_by(
                const std::string & input,
                char splitter);

        template<typename T>
        static T deg_to_rad(T deg)
        {
            constexpr double multiplier = M_PI / 180.0;
            return static_cast<T>(deg * multiplier);
        }
    };

}  // namespace mapora::utils


#endif //BUILD_UTILS_HPP
