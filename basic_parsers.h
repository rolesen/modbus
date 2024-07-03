#pragma once


#include <stdlib.h>

#include <string_view>
#include <array>

#include <limits>
#include <cstdint>
#include <gsl/span>

#include "Memory.h"


namespace {

constexpr size_t err_pos = std::numeric_limits<size_t>::max();

struct hexstr {
    hexstr(std::string_view strview) : hexstr_{strview} {
    }
    
    std::string_view hexstr_;

    size_t next_ = 0;
    
    template<class T>
    size_t get(T& val, size_t pos) const noexcept {
        std::array<char, sizeof(uint64_t)*2+1/*'\0'*/> strbuf_{};

        if ((hexstr_.size() < (pos + 2 *sizeof(T))) || (pos == err_pos)) {
            return err_pos;
        }

        for (size_t i = 0; i < 2 *sizeof(T); ++i) {
            strbuf_[i] = hexstr_[i + pos];
        }

        val = strtoull(strbuf_.data(), nullptr, 16);
        return pos + sizeof(T) * 2;
    }

    template<class T>
    friend hexstr& operator>>(hexstr& lhs, T& rhs) {
        lhs.next_ = lhs.get(rhs, lhs.next_);
        return lhs;
    }
    
};

template<bool swap_endianess>
struct datastr {
    datastr(gsl::span<const char> span) : datastr_{span} {
    }
    
    gsl::span<const char> datastr_;

    size_t next_ = 0;
    
    template<class T>
    size_t get(T& value, size_t pos) const {
        if ((datastr_.size() < (pos + sizeof(T))) || (pos == err_pos)) {
            return err_pos;
        }
        
        value = memory<T>(this->datastr_.data() + pos);

        if constexpr (swap_endianess) {
            value = byteswap(value);
        }

        return pos + sizeof(T);
    }

    template<class T>
    friend datastr& operator>>(datastr& lhs, T& rhs) {
        lhs.next_ = lhs.get(rhs, lhs.next_);
        if(lhs.next_ == err_pos) {
            rhs = T{};
        }
        return lhs;
    }
    
};

}//namespace
