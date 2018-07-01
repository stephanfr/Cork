#pragma once

/*
	Stephan Friedl
	Derived from:

	Written in 2016 by David Blackman and Sebastiano Vigna (vigna@acm.org)

	To the extent possible under law, the author has dedicated all copyright
	and related and neighboring rights to this software to the public domain
	worldwide. This software is distributed without any warranty.

	See <http://creativecommons.org/publicdomain/zero/1.0/>.
*/


/*
	A note on Xoroshiro128Plus:

	This is not a perfect PRNG but it is very good and very fast.  For non-cryptographic and
	not long-running sequences (it only has 128 bits of internal state) it should be perfectly fine.
	If you have some application in which you will be doing very fine-grained statistics on very
	long running simulations using a huge number of generated random values, you may want to
	look into a different, more statistically robust generator with a bigger internal state variable.

	That said, anything is better than the C Lib rand().
*/



#include <stdint.h>

#include <array>



class SplitMix64
{
public:

	SplitMix64(const uint64_t		state)
		: m_state(state)
	{}

	uint64_t next()
	{
		uint64_t z = (m_state += UINT64_C(0x9E3779B97F4A7C15));
		z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
		z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);

		return(z ^ (z >> 31));
	}

private :

	uint64_t		m_state;
};



class Xoroshiro128Plus
{
public :

	Xoroshiro128Plus(const uint64_t		seed)
	{
		SplitMix64		splitMix(seed);

		m_state[0] = splitMix.next();
		m_state[1] = splitMix.next();
	}

	Xoroshiro128Plus(const std::array<uint64_t,2>		seed )
	{
		m_state[0] = seed[0];
		m_state[1] = seed[1];
	}


	uint64_t		next(void)
	{
		const uint64_t		s0 = m_state[0];
		uint64_t			s1 = m_state[1];
		const uint64_t		nextRandomValue = s0 + s1;

		s1 ^= s0;
		m_state[0] = rotl(s0, 55) ^ s1 ^ (s1 << 14);		// a, b
		m_state[1] = rotl(s1, 36);							// c

		return( nextRandomValue );
	}



	//	This is the jump function for the generator. It is equivalent
	//		to 2^64 calls to next(); it can be used to generate 2^64
	//		non-overlapping subsequences for parallel computations.

	void			jump(void)
	{
		static const uint64_t JUMP[] = { 0xbeac0467eba5facb, 0xd86b048b86aa9922 };

		uint64_t s0 = 0;
		uint64_t s1 = 0;

		for (int i = 0; i < sizeof JUMP / sizeof *JUMP; i++)
		{
			for (int b = 0; b < 64; b++)
			{
				if (JUMP[i] & UINT64_C(1) << b)
				{
					s0 ^= m_state[0];
					s1 ^= m_state[1];
				}

				next();
			}
		}

		m_state[0] = s0;
		m_state[1] = s1;
	}


private :

	uint64_t		m_state[2];


	inline uint64_t rotl(const uint64_t x, int k)
	{
		return (x << k) | (x >> (64 - k));
	}

};


