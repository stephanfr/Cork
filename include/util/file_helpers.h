#pragma once

#include <filesystem>
#include <fstream>
#include <iterator>

namespace Cork::Files
{
    class LineByLineFileReader
    {
       public:
        static constexpr size_t DEFAULT_BUFFER_SIZE = 1024;

        LineByLineFileReader(std::filesystem::path file_path, size_t buffer_size = DEFAULT_BUFFER_SIZE)
            : file_path_(std::move(file_path)), buffer_size_(buffer_size)
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
        bool    read_line_exactly( const char* format_string, int num_fields, Args ... args )
        {
            next_line_into_string_buffer();

            if (!good())
            {
                return false;
            }

            std::string     full_format_string( format_string );
            full_format_string += " %n";

            int chars_processed;

            auto fields_processed = sscanf(string_buffer_.c_str(), full_format_string.c_str(), args ..., &chars_processed);

            if ((fields_processed != num_fields) || (chars_processed != string_buffer_.length()))
            {
                return false;
            }

            return true;
        }

       private:
        const std::filesystem::path file_path_;
        size_t buffer_size_;

        std::ifstream input_stream_;

        std::unique_ptr<char> char_buffer_;
        std::string string_buffer_;

        void next_line_into_string_buffer()
        {
            input_stream_ >> std::ws;
            input_stream_.getline(char_buffer_.get(), 60);

            string_buffer_ = char_buffer_.get();

            string_buffer_.erase(std::find_if(string_buffer_.rbegin(), string_buffer_.rend(),
                                              [](unsigned char ch) { return !std::isspace(ch); })
                                     .base(),
                                 string_buffer_.end());
        }

    };
}  // namespace Cork::Files
