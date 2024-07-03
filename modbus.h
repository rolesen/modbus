#pragma once

#include "basic_parsers.h"

#include "Memory.h"

#include <stdexcept>

enum class modbus_program_code : uint8_t{
    read_input_registers = 4,
    read_multiple_holding_registers = 3,

    write_single_coil = 5,
    write_single_holding_register = 6,
};


using modbus_error = uint8_t;


enum modbus_error_code : uint8_t { // FIXME: get rid of reserved, enum was copied from BatErrorCode, 
    MOD_NO_ERROR,
    MOD_RESERVED_1,
    MOD_UNEXPECTED,
    MOD_BUFFER_LEN_ERROR,
    MOD_PAYLOAD_LEN,
    MOD_VERSION,
    MOD_LCHECKSUM,
    MOD_CHECKSUM,
    MOD_RESERVED_2,
    MOD_MISMATCH_HEADER,
    MOD_RESERVED_3,
    MOD_MISSING_DATA,

    MOD_VENDER_CODE_OFFSET = 0x80 // Add to vendor's error code
};


namespace {
    template<class T, size_t N, size_t M>
    constexpr void array_copy(std::array<T, N> from, T(&target_arr)[M]) {
        static_assert(N <= M);
        std::copy(std::begin(from), std::end(from), std::begin(target_arr));
    }

    namespace details{
        template<class T, size_t N, class InputIt, size_t... I>
        constexpr std::array<T, N> make_array_impl(InputIt first, std::index_sequence<I...>) {
            return { {first[I] ...} };
        }
    }


    template<class T, size_t N, class InputIt>
    constexpr std::array<T, N> make_array(InputIt first) {
        return details::make_array_impl<T, N>(first, std::make_index_sequence<N>());
    }



    inline unsigned short CRC16_modbus(const unsigned char *addr, int num) {
        unsigned short crc = 0xFFFF;
        int i;
        while (num--) {
            crc ^= *addr++;
            for (i = 0; i < 8; i++) {
                if (crc & 1) {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }


    inline std::string to_string(modbus_error_code error_code) 
    {
        switch (error_code) {
            case MOD_NO_ERROR: return "none";
            case MOD_RESERVED_1: return "reserved 1";
            case MOD_UNEXPECTED: return "unexpected modbus error";
            case MOD_BUFFER_LEN_ERROR: return "modbus buffer length error";
            case MOD_PAYLOAD_LEN: return "modbus payload length error";
            case MOD_VERSION: return "modbus mismatch version error";
            case MOD_LCHECKSUM: return "modbus lchecksum eror";
            case MOD_CHECKSUM: return "modbus checksum error";
            case MOD_RESERVED_2: return "modbus reserved 2 error";
            case MOD_MISMATCH_HEADER: return "modbus mismatch error";
            case MOD_RESERVED_3: return "modbus reserved 3 error";
            case MOD_MISSING_DATA: return "invalid frame or no data error";
            default: {
                if (error_code > MOD_VENDER_CODE_OFFSET) {
                    return std::string("modbus vendor exception code: ") + std::to_string(int{error_code});
                }
            }
        }
        return "unexpected error code";
    }


    inline std::array<char, 8> modbus_rtu_encode_request_16_16(uint8_t station_addr, uint8_t function_code, uint16_t first, uint16_t second) {
        std::array<char, 8> buf;
        buf[0] = station_addr;
        buf[1] = static_cast<uint8_t>(function_code);
        memory<uint16_t>(&buf[2]) = byteswap(first);
        memory<uint16_t>(&buf[4]) = byteswap(second);
        memory<uint16_t>(&buf[6]) = CRC16_modbus(reinterpret_cast<const unsigned char*>(&buf), 6);
        return buf;
    }


    //Decodes a multi-data response. In case of error, returns an error_code (ie. !=0) and 'data' is reset to zero/default.
    //All 'data' points has to be given to match size_in_bytes or the program is malformed.
    const uint16_t* raw_decode_response_multi(uint8_t station_address, modbus_program_code function_code, uint16_t number_of_registers, gsl::span<const char> frame_data, modbus_error_code& error) {
        error = modbus_error_code{};//reset error code

        if (frame_data.size() < 5) {
            error = MOD_MISSING_DATA;
            return nullptr;
        }
        

        datastr<true/*swap endianness*/> stream(frame_data);
        uint8_t rec_station_addr{};
        uint8_t rec_function_code{};

        stream >> rec_station_addr;
        stream >> rec_function_code;

        if (rec_function_code > 0x80) {
            //modbus exception_code in response
            uint8_t rec_exception_code{};
            stream >> rec_exception_code;
            error = static_cast<modbus_error_code>(MOD_VENDER_CODE_OFFSET + (rec_exception_code % 0x80));
            return nullptr;
        }

        uint8_t size;
        stream >> size;

        if (station_address != rec_station_addr || static_cast<uint8_t>(function_code) != rec_function_code || size != number_of_registers * 2) {
            error = MOD_MISMATCH_HEADER;
            return nullptr;
        }
        
        const void* data_start_ptr = stream.datastr_.data() + stream.next_;//store start of data pointer for later
        uint16_t tmp{};
        for (int i = 0; i < number_of_registers; ++i) {
            stream >> tmp;//skipping data section for now (this will get incorporated later)
        }

        //need to byteswap crc to match:
        if (stream.next_ == err_pos) {
            error = MOD_PAYLOAD_LEN;
            return nullptr;
        }

        uint16_t crc = byteswap(CRC16_modbus(reinterpret_cast<const unsigned char*>(stream.datastr_.data()), stream.next_ ));
        uint16_t rec_crc;
        stream >> rec_crc;

        if (rec_crc != crc) {
            error = MOD_CHECKSUM;
            return nullptr;
        }

        if (stream.next_ == err_pos) {
            error = MOD_BUFFER_LEN_ERROR;
            return nullptr;
        }
        (void)error;

        return static_cast<const uint16_t*>(data_start_ptr);
    }

}//namespace


class modbus_exception : public std::runtime_error {
public:
    modbus_exception(modbus_error_code error_code) : std::runtime_error(to_string(error_code)), modbus_error_code_(error_code) {
    }

    modbus_error_code code() {return modbus_error_code_;}

private:
    modbus_error_code modbus_error_code_;
};




template <uint16_t from_data_address, uint16_t to_data_address, modbus_program_code function_code, bool swapbytes = false>
class modbus_rtu_multi {
public:
    static_assert( function_code == modbus_program_code::read_input_registers || function_code == modbus_program_code::read_multiple_holding_registers);
    static_assert(from_data_address <= to_data_address);
    static constexpr const uint16_t number_of_registers = 1 + (to_data_address - from_data_address);
    static_assert(number_of_registers <= 125);//rtu protocol max
    static constexpr const size_t recv_buffer_size = 1/*address*/ + 1/*function code*/ + (1/*data size field*/ + 2 * number_of_registers)/*data field*/ + 2/*CRC*/;
    
    static constexpr uint16_t base_address = from_data_address;

    class register_data_view {
    public:
        register_data_view() {//constructor representing empty register data view result
        }
        register_data_view(const uint16_t* data_ptr) : data_{make_array<const uint16_t, number_of_registers>(data_ptr)} {
        }

        template<uint16_t register_addr>
        uint16_t get() const {
            return byteswap(get_impl<register_addr>());
        }

        template<uint16_t register_addr, class T>
        T get() const requires (sizeof(T) % 2 == 0) {
            return byteswap(get_composite_impl<register_addr, T>(std::make_index_sequence<sizeof(T) / 2>()));
        }

        template<uint16_t register_addr, uint16_t register_addr_last>
        std::array<uint16_t, 1 + register_addr_last - register_addr> get_multi() {
            
        }

        explicit operator bool() const noexcept {return !data_.empty();}

        template<class OS>
        friend OS& operator<<(OS& os, const register_data_view& view) {
            os << "[";
            for (auto d : view.data_) {
                os << d << ", ";
            }
            os << "]";
            return os;
        }

        auto& data() {return data_;}

    private:
        template<uint16_t register_addr>
        uint16_t get_impl() const {
            if (data_.empty()) {
                return 0;
            }
            static_assert(register_addr >= from_data_address, "Register address outside range");
            constexpr auto index = register_addr - from_data_address;
            static_assert(index < number_of_registers, "Register address outside range");
            return swapbytes ? byteswap(data_[index]) : data_[index];
        }

        template<uint16_t register_addr, class T, size_t... I>
        T get_composite_impl(std::index_sequence<I...>) const {
            std::array<uint16_t, sizeof(T) / 2> value{};
            ([this, &value]{std::get<I>(value) = get_impl<register_addr + I>(); }(), ...);
            return std::bit_cast<T>(value);
        }

        std::array<const uint16_t, number_of_registers> data_{};
    };

    //Decodes a multi-data response. In case of error, returns an error_code (ie. !=0) and 'data' is reset to zero/default.
    //All 'data' points has to be given to match size_in_bytes or the program is malformed.
    register_data_view decode_response(gsl::span<const char> frame_data, modbus_error_code& error_code) const {
        const uint16_t* data_ptr = ::raw_decode_response_multi(station_address, function_code, number_of_registers, frame_data, error_code);
        return data_ptr ? register_data_view{data_ptr} : register_data_view{};
    }

    //throwing version
    register_data_view decode(gsl::span<const char> frame_data) const {
        modbus_error_code error{};
        auto res = decode_response(frame_data, error);
        if (error != MOD_NO_ERROR) {
            throw modbus_exception(error);
        }
        return res;
    }


    template<size_t N>
    size_t encode_request(char (&buf)[N]) const {
        static_assert(N >= 8);
        auto arr = encode();
        array_copy(arr, buf);
        return 8;
    }

    std::array<char, 8> encode() const {
        return modbus_rtu_encode_request_16_16(station_address, static_cast<uint8_t>(function_code), from_data_address, number_of_registers);
    }

    uint8_t station_address{0};
};


template <uint16_t data_address, modbus_program_code function_code>
class modbus_rtu_write_single {
public:
    static_assert( function_code == modbus_program_code::write_single_holding_register || function_code == modbus_program_code::write_single_coil);

    static constexpr const size_t recv_buffer_size = 8;

    using value_type = std::conditional_t<function_code == modbus_program_code::write_single_coil, bool, uint16_t>;

    std::array<char, 8> encode(value_type value) const {
        if constexpr (std::is_same_v<value_type, bool>) {
            return modbus_rtu_encode_request_16_16(station_address, static_cast<uint8_t>(function_code), data_address, value ? 0xFF00 : 0);
        }
        else {
            return modbus_rtu_encode_request_16_16(station_address, static_cast<uint8_t>(function_code), data_address, value);
        }
    }

    template<size_t N>
    size_t encode_request(char (&buf)[N], value_type value) const {
        static_assert(N >= 8);
        auto arr = encode(station_address, value);
        array_copy(arr, buf);
        return 8;
    }

    void decode_response(gsl::span<const char> frame_data, modbus_error_code& error) const {
        error = modbus_error_code{};//reset error code

        datastr<true/*swap endianness*/> stream(frame_data);

        uint8_t rec_station_addr{};
        uint8_t rec_function_code{};

        stream >> rec_station_addr;
        stream >> rec_function_code;



        if (rec_function_code > 0x80) {
            //modbus exception_code in response
            uint8_t rec_exception_code{};
            stream >> rec_exception_code;
            error = static_cast<modbus_error_code>(MOD_VENDER_CODE_OFFSET + (rec_exception_code % 0x80));
            return;
        }
        if (rec_function_code != static_cast<uint8_t>(function_code) || station_address != rec_station_addr) {
            error = MOD_MISMATCH_HEADER;
            return;
        }

        uint16_t data_addr{};
        uint16_t value{};

        stream >> data_addr;
        stream >> value;

        //need to byteswap crc to match:
        if (stream.next_ == err_pos) {
            error = MOD_BUFFER_LEN_ERROR;
            return;
        }

        uint16_t crc = byteswap(CRC16_modbus(reinterpret_cast<const unsigned char*>(stream.datastr_.data()), stream.next_ ));
        uint16_t rec_crc;
        stream >> rec_crc;

        if (rec_crc != crc) {
            error = MOD_CHECKSUM;
            return;
        }

        if (stream.next_ == err_pos) {
            error = MOD_BUFFER_LEN_ERROR;
            return;
        }
    }

    //throwing version
    void decode(gsl::span<const char> frame_data) const {
        modbus_error_code error{};
        decode_response(frame_data, error);
        if (error != MOD_NO_ERROR) {
            throw modbus_exception(error);
        }
    }

    uint8_t station_address{0};
};

//convenience / short hand commands
template <uint16_t from_data_address, uint16_t to_data_address>
using modbus_read_inputs = modbus_rtu_multi<from_data_address, to_data_address, modbus_program_code::read_input_registers>;

template <uint16_t from_data_address, uint16_t to_data_address>
using modbus_read_registers =  modbus_rtu_multi<from_data_address, to_data_address, modbus_program_code::read_multiple_holding_registers>;

template <uint16_t data_address>
using modbus_write_register = modbus_rtu_write_single<data_address, modbus_program_code::write_single_holding_register>;

template <uint16_t data_address>
using modbus_write_coil = modbus_rtu_write_single<data_address, modbus_program_code::write_single_coil>;



