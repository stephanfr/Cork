#pragma once

namespace SEFUtility::CompileTime
{
    class conststr
    {
       public:
        template <std::size_t N>
        constexpr conststr(const char (&a)[N]) : p_(a), sz_(N - 1)
        {}

        //  In C++11, constexpr expressions signal errors by throwing exceptions from the conditional operator ?:

        constexpr char operator[](std::size_t n) const { return n < sz_ ? p_[n] : throw std::out_of_range(""); }
        constexpr operator const char*() { return p_; }
        constexpr std::size_t size() const { return sz_; }

       private:
        const char* p_;
        std::size_t sz_;
    };

    constexpr unsigned count_char_occurances(conststr str, char c, unsigned i = 0, unsigned ans = 0)
    {
        return i == str.size() ? ans
                               : str[i] == c ? count_char_occurances(str, c, i + 1, ans + 1)
                                             : count_char_occurances(str, c, i + 1, ans);
    }
}  // namespace SEFUtility::CompileTime
