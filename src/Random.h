#ifndef INCL_RANDOM_H
#define INCL_RANDOM_H

#include <esp_system.h>
#include <HardwareSerial.h>

class Random {
    uint64_t seed;
public:
    Random() {
        seed = esp_random();
    }
    Random(uint64_t seed) {
        setSeed(seed);
    }
    void setSeed(uint64_t seed) {
        this->seed = (seed ^ 0x5DEECE66DL) & ((1LL << 48) - 1);
    }
    uint32_t next(int bits) {
        seed = (seed * 0x5DEECE66DL + 0xBL) & ((1LL << 48) - 1);
        return (uint32_t)(seed >> (48 - bits));
    }
    int nextIntX() {
        return (int)next(32);
    }
    unsigned nextUInt() {
        return (unsigned)next(32);
    }

    bool bernoulli(uint32_t p, uint32_t max) {
        uint64_t thr = (uint32_t)((uint64_t)p * 0x7fffffff / max);
        return (next(31) < thr);
    }

    // https://www.pcg-random.org/posts/bounded-rands.html
    int nextInt(int min, int max) {
        uint32_t range = max - min;
        uint32_t x = next(32);
        uint64_t m = uint64_t(x) * uint64_t(range);
        uint32_t l = uint32_t(m);
        if (l < range) {
            uint32_t t = -range;
            if (t >= range) {
                t -= range;
                if (t >= range)
                    t %= range;
            }
            while (l < t) {
                x = next(32);
                m = uint64_t(x) * uint64_t(range);
                l = uint32_t(m);
            }
        }
        return min + (int)(m >> 32);
    }

    int64_t next64() {
        return ((uint64_t)next(32) << 32) + next(32);
    }
    double nextDouble() {
        return (((uint64_t)next(26) << 27) + next(27)) / (double)(1LL << 53);
    }
};


class RandomExp {
    float beta;
    int *expDistribTable;
    int expDistribTableCnt;
public:
    RandomExp(float beta) {
        expDistribTableCnt = 16;
        expDistribTable = new int[expDistribTableCnt];
        beta = 0;
        setScale(beta);
    }

    void setScale(float newBeta) {
        if (newBeta == beta) {
            return;
        }

        beta = newBeta;
        for (int i = 0; i < expDistribTableCnt; i++) {
            double expVal = -log((i + 1) / (double)expDistribTableCnt) * beta;
            if (expVal < 1) {
                expDistribTable[i] = 1;
            } else if (expVal >= 10 * beta) {
                expDistribTable[i] = 10 * beta;
            } else {
                expDistribTable[i] = (int)(expVal + 0.5);
            }
        }
    }

    uint32_t getRndExp(Random &rnd)
    {
        const uint32_t sz = expDistribTableCnt;
        uint32_t r = rnd.nextUInt() % (sz << 4);
        uint32_t slot = (r >> 4);
        uint32_t dec = (r & 0x0F);
        uint32_t result;
        if (slot < sz - 1) {
            result = (expDistribTable[slot] * dec + expDistribTable[slot + 1] * (16 - dec)) >> 4;
        } else {
            result = expDistribTable[slot];
        }
        return result;
    }

};

#endif

