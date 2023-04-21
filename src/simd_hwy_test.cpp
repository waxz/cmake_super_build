//
// Created by waxz on 4/13/23.
//
//#include "hwy/foreach_target.h"  // IWYU pragma: keep

// Must come after foreach_target.h to avoid redefinition errors.
#include "hwy/aligned_allocator.h"
#include "hwy/highway.h"
#include "hwy/nanobenchmark.h"


#ifndef HWY_ASSERT_EQ

#define HWY_ASSERT_EQ(expected, actual)                                     \
  hwy::AssertEqual(expected, actual, hwy::TargetName(HWY_TARGET), __FILE__, \
                   __LINE__)

#define HWY_ASSERT_ARRAY_EQ(expected, actual, count)                          \
  hwy::AssertArrayEqual(expected, actual, count, hwy::TargetName(HWY_TARGET), \
                        __FILE__, __LINE__)

#define HWY_ASSERT_STRING_EQ(expected, actual)                          \
  hwy::AssertStringEqual(expected, actual, hwy::TargetName(HWY_TARGET), \
                         __FILE__, __LINE__)

#define HWY_ASSERT_VEC_EQ(d, expected, actual) \
  AssertVecEqual(d, expected, actual, __FILE__, __LINE__)

#define HWY_ASSERT_MASK_EQ(d, expected, actual) \
  AssertMaskEqual(d, expected, actual, __FILE__, __LINE__)

#endif  // HWY_ASSERT_EQ




HWY_BEFORE_NAMESPACE();

namespace hn = hwy::HWY_NAMESPACE;

using T = float;

void MulAddLoop(const T* HWY_RESTRICT mul_array,
                const T* HWY_RESTRICT add_array,
                const size_t size, T* HWY_RESTRICT x_array) {
    const hn::ScalableTag<T> d;
    for (size_t i = 0; i < size; i += hn::Lanes(d)) {
        const auto mul = hn::Load(d, mul_array + i);
        const auto add = hn::Load(d, add_array + i);
        auto x = hn::Load(d, x_array + i);
        x = hn::MulAdd(mul, x, add);
        hn::Store(x, d, x_array + i);
    }
}


namespace hwy{

// The maximum vector size used in tests when defining test data. DEPRECATED.
    constexpr size_t kTestMaxVectorSize = 64;

// 64-bit random generator (Xorshift128+). Much smaller state than std::mt19937,
// which triggers a compiler bug.
    class RandomState {
    public:
        explicit RandomState(const uint64_t seed = 0x123456789ull) {
            s0_ = SplitMix64(seed + 0x9E3779B97F4A7C15ull);
            s1_ = SplitMix64(s0_);
        }

        HWY_INLINE uint64_t operator()() {
            uint64_t s1 = s0_;
            const uint64_t s0 = s1_;
            const uint64_t bits = s1 + s0;
            s0_ = s0;
            s1 ^= s1 << 23;
            s1 ^= s0 ^ (s1 >> 18) ^ (s0 >> 5);
            s1_ = s1;
            return bits;
        }

    private:
        static uint64_t SplitMix64(uint64_t z) {
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            return z ^ (z >> 31);
        }

        uint64_t s0_;
        uint64_t s1_;
    };

    static HWY_INLINE uint32_t Random32(RandomState* rng) {
        return static_cast<uint32_t>((*rng)());
    }

    static HWY_INLINE uint64_t Random64(RandomState* rng) { return (*rng)(); }
}
// Calls function defined in skeleton-inl.h.
struct TestSumMulAdd {
    template <class T, class D>
    HWY_NOINLINE void operator()(T /*unused*/, D d) {
        hwy::RandomState rng;
        const size_t count = 4096;
        EXPECT_EQ(0, count % hn::Lanes(d));
        auto mul = hwy::AllocateAligned<T>(count);
        auto x = hwy::AllocateAligned<T>(count);
        auto add = hwy::AllocateAligned<T>(count);
        for (size_t i = 0; i < count; ++i) {
            mul[i] = static_cast<T>(Random32(&rng) & 0xF);
            x[i] = static_cast<T>(Random32(&rng) & 0xFF);
            add[i] = static_cast<T>(Random32(&rng) & 0xFF);
        }
        double expected_sum = 0.0;
        for (size_t i = 0; i < count; ++i) {
            expected_sum += mul[i] * x[i] + add[i];
        }

        MulAddLoop(d, mul.get(), add.get(), count, x.get());
//        HWY_ASSERT_EQ(4344240.0, expected_sum);
    }
};




namespace myproject {
    namespace HWY_NAMESPACE {
        HWY_ATTR void MyFunc(float* HWY_RESTRICT out) {

            
        }
    } // namespace HWY_NAMESPACE
} // namespace myproject
HWY_AFTER_NAMESPACE();

int main(){
    TestSumMulAdd test;
}