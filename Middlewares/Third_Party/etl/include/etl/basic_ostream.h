///\file

/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2020 John Wellbelove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#ifndef ETL_BASIC_OSTREAM_INCLUDED
#define ETL_BASIC_OSTREAM_INCLUDED

///\ingroup iostream

#include "etl/platform.h"
#include "etl/to_string.h"
#include <stdio.h>

namespace etl
{
struct endl_t {
};
static ETL_CONSTANT endl_t endl;

template <typename TIString, typename TStringView>
class basic_ostream
{
public:
    typedef TIString istring_type;
    typedef typename TIString::value_type value_type;
    typedef typename TIString::pointer pointer;
    typedef typename TIString::const_pointer const_pointer;

    //*************************************************************************
    /// Construct.
    //*************************************************************************
    explicit basic_ostream(TIString& text_)
        : text(text_)
    {
    }

    //*********************************
    /// From endl
    //*********************************
    friend basic_ostream& operator<<(basic_ostream& ss, endl_t l)
    {
#ifdef DEBUG
        printf("\n");
#endif
        return ss;
    }

    //*********************************
    /// From a string view
    //*********************************
    friend basic_ostream& operator<<(basic_ostream& ss, TStringView view)
    {
#ifdef DEBUG
        printf("%s", view.data());
#endif
        return ss;
    }

    //*********************************
    /// From a character pointer to a string
    //*********************************
    friend basic_ostream& operator<<(basic_ostream& ss, pointer p)
    {
#ifdef DEBUG
        printf("%s", p);
#endif
        return ss;
    }

    //*********************************
    /// From a const character pointer to a string
    //*********************************
    friend basic_ostream& operator<<(basic_ostream& ss, const_pointer p)
    {
#ifdef DEBUG
        printf("%s", p);
#endif
        return ss;
    }

    //*********************************
    /// From a string interface
    //*********************************
    friend basic_ostream& operator<<(basic_ostream& ss, const TIString& text)
    {
#ifdef DEBUG
        printf("%s", text.data());
#endif
        return ss;
    }

    //*********************************
    /// From a string
    //*********************************
    template <template <size_t> class TString, size_t SIZE>
    friend basic_ostream& operator<<(basic_ostream& ss, const TString<SIZE>& text)
    {
#ifdef DEBUG
        printf("%s", text.data());
#endif
        return ss;
    }

    //*********************************
    /// From anything else
    //*********************************
    template <typename T>
    friend basic_ostream& operator<<(basic_ostream& ss, const T& value)
    {
#ifdef DEBUG
        etl::to_string(value, ss.text, etl::format_spec());
        printf("%s", ss.text.data());
#endif
        return ss;
    }

private:
    TIString& text;

    basic_ostream(const basic_ostream&) ETL_DELETE;
    basic_ostream& operator=(const basic_ostream&) ETL_DELETE;
};
}  // namespace etl

#endif
