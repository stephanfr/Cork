/*
Copyright (c) 2013 Stephan Friedl

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above copyright holders
shall not be used in advertising or otherwise to promote the sale, use or other
dealings in this Software without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
 */

#pragma once


#include <boost\align\aligned_delete.hpp>
#include <memory>




template<class T>
using aligned_unique_ptr = std::unique_ptr<T,boost::alignment::aligned_delete>;



template<class T, class... Args>
inline aligned_unique_ptr<T> make_aligned(Args&&... args)
{
    auto p = boost::alignment::aligned_alloc(__alignof(T), sizeof(T));

    if (!p)
	{
        throw std::bad_alloc();
    }
    
	try
	{
        auto q = ::new(p) T(std::forward<Args>(args)...);
        return aligned_unique_ptr<T>(q);
    }
	catch (...)
	{
        boost::alignment::aligned_free(p);
        throw;
    }
}

