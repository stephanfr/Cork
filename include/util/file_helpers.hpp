#pragma once

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "compile_time_utilities.hpp"

namespace Cork::Files
{
    //
    //  The next class is a bit hard to follow - it has some compile time functions as well
    //      mixed in to make the interface little more robust when literals are used for the format string.
    //

    class LineByLineFileReader
    {
       public:
        static constexpr size_t DEFAULT_BUFFER_SIZE = 1024;

        LineByLineFileReader(std::filesystem::path file_path, size_t buffer_size = DEFAULT_BUFFER_SIZE,
                             std::string comment_delimiter = "")
            : file_path_(std::move(file_path)), buffer_size_(buffer_size), comment_delimiter_(comment_delimiter)
        {
            char_buffer_.reset(static_cast<char*>(malloc(buffer_size)));
            string_buffer_.reserve(buffer_size);

            input_stream_.open(file_path_);
        }

        bool good() const { return input_stream_.good(); }

        //  Returns a front and rear trimmed string which is the next line from the file.

        const std::string& next_line()
        {
            next_line_into_string_buffer();

            return string_buffer_;
        }

        template <typename... Args>
        bool read_line_exactly(SEFUtility::CompileTime::conststr format_string, Args... args)
        {
            int num_fields = SEFUtility::CompileTime::count_char_occurances(format_string, '%');

            return read_line_exactly_internal(format_string, num_fields, args...);
        }

        template <typename... Args>
        bool read_line_exactly(const std::string& format_string, int num_fields, Args... args)
        {
            return read_line_exactly_internal(format_string.c_str(), num_fields, args...);
        }

       private:
        template <typename T>
        struct FreeDeleter
        {
            void operator()(T* pointer) { free(pointer); }
        };

        const std::filesystem::path file_path_;
        size_t buffer_size_;

        std::ifstream input_stream_;

        std::string comment_delimiter_;

        std::unique_ptr<char, FreeDeleter<char>> char_buffer_;
        std::string string_buffer_;

        void next_line_into_string_buffer()
        {
            do
            {
                input_stream_ >> std::ws;
                input_stream_.getline(char_buffer_.get(), buffer_size_ - 1);

                string_buffer_ = char_buffer_.get();

                string_buffer_.erase(std::find_if(string_buffer_.rbegin(), string_buffer_.rend(),
                                                  [](unsigned char ch) { return !std::isspace(ch); })
                                         .base(),
                                     string_buffer_.end());

                if (!comment_delimiter_.empty())
                {
                    if (auto comment_loc = string_buffer_.find(comment_delimiter_); comment_loc != std::string::npos)
                    {
                        string_buffer_.erase(comment_loc);
                    }
                }
            } while (string_buffer_.empty() && !input_stream_.eof());
        }

        template <typename... Args>
        bool read_line_exactly_internal(const char* format_string, int num_fields, Args... args)
        {
            next_line_into_string_buffer();

            if (!good())
            {
                return false;
            }

            std::string full_format_string(format_string);
            full_format_string += " %n";

            int chars_processed;

            auto fields_processed =
                std::sscanf(string_buffer_.c_str(), full_format_string.c_str(), args..., &chars_processed);

            if ((fields_processed != num_fields) || (chars_processed != string_buffer_.length()))
            {
                return false;
            }

            return true;
        }
    };

}  // namespace Cork::Files
