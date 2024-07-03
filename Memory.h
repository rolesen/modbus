#pragma once

#include <bit>


static_assert( __cpp_lib_bit_cast);


namespace {

    template<class T>
    volatile T& memory(char* ptr) {
        return *reinterpret_cast<T*>(ptr);
    };

    template<class T>
    volatile const T& memory(const char* ptr) {
        return *reinterpret_cast<const T*>(ptr);
    };

    template<class T>
    volatile T& memory(std::byte* ptr) {
        return *reinterpret_cast<T*>(ptr);
    };

    template<class T>
    volatile const T& memory(const std::byte* ptr) {
        return *reinterpret_cast<const T*>(ptr);
    };


    template<class T>
    T byteswap(T val) {
        if constexpr(sizeof(T) == 1) {
            return val;
        }
        else if constexpr(sizeof(T) == 2) {
            uint16_t r = __builtin_bswap16(std::bit_cast<uint16_t>(val));
            return std::bit_cast<T>(r);
        }
        else if constexpr(sizeof(T) == 4) {
            uint32_t r = __builtin_bswap32(std::bit_cast<uint32_t>(val));
            return std::bit_cast<T>(r);
        }
        else if constexpr(sizeof(T) == 8) {
            uint64_t r = __builtin_bswap64(std::bit_cast<uint64_t>(val));
            return std::bit_cast<T>(r);
        }
        else {
            static_assert(sizeof(T) == 0);
        }
    }

}

