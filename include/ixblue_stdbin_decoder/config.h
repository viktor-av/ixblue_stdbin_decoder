#pragma once

#define NO_BOOST 1

#ifdef NO_BOOST

#include <endian.h>
#define BIG_TO_NATIVE64 be64toh
#define BIG_TO_NATIVE32 be32toh
#define BIG_TO_NATIVE16 be16toh

#else //NO_BOOST

#include <boost/endian/conversion.hpp>
#define BIG_TO_NATIVE64 boost::endian::big_to_native
#define BIG_TO_NATIVE32 boost::endian::big_to_native
#define BIG_TO_NATIVE16 boost::endian::big_to_native

#endif //NO_BOOST
