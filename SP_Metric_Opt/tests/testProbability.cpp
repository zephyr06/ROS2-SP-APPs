// #include <gtest/gtest.h>

#include "gmock/gmock.h"  // Brings in gMock.
#include "sources/Safety_Performance_Metric/Probability.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/readwrite.h"
using ::testing::AtLeast;  // #1
using ::testing::Return;
using namespace std;
using namespace SP_OPT_PA;
using namespace GlobalVariables;
TEST(CDF, v1) {
    GaussianDist gau_dis(3, 1);
    EXPECT_EQ(0.5, gau_dis.CDF(3));
}
TEST(FiniteDist, equal) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(0, 0.9),
                                          Value_Proba(4, 0.1)};
    std::vector<Value_Proba> dist_vec3 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};

    std::vector<Value_Proba> dist_vec1_approx = {Value_Proba(3.1, 0.099),
                                                 Value_Proba(6.9, 0.901)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);
    FiniteDist dist3(dist_vec3);
    FiniteDist dist1_approx(dist_vec1_approx);

    EXPECT_TRUE(dist1 == dist3);
    EXPECT_FALSE(dist1 == dist2);
    EXPECT_TRUE(dist1 != dist2);

    EXPECT_TRUE(dist1 == dist1_approx);
}

// approx_equal must honor its `tolerance` arg: today the per-sample comparison
// falls through to Value_Proba::operator== (hardcoded 1e-1) and min/max uses
// near() (absolute 1e-6), so the parameter is a no-op. This test pins the fix.
TEST(FiniteDist, approx_equal_respects_tolerance) {
    std::vector<Value_Proba> v1 = {Value_Proba(10.0, 0.5),
                                   Value_Proba(20.0, 0.5)};
    std::vector<Value_Proba> v2 = {Value_Proba(10.0, 0.5),
                                   Value_Proba(20.0, 0.5)};
    FiniteDist d1(v1);
    FiniteDist d2(v2);
    // identical -> equal at any tolerance
    EXPECT_TRUE(d1.approx_equal(d2, 1e-9));
    EXPECT_FALSE(d1.approx_not_equal(d2, 1e-9));

    // 5% relative drift on value 20 -> 21. prob unchanged.
    std::vector<Value_Proba> v3 = {Value_Proba(10.0, 0.5),
                                   Value_Proba(21.0, 0.5)};
    FiniteDist d3(v3);
    // 5% drift: inside a loose 1e-1 tol -> equal; outside a tight 1e-3 -> not.
    EXPECT_TRUE(d1.approx_equal(d3, 1e-1));
    EXPECT_FALSE(d1.approx_equal(d3, 1e-3));

    // min/max: near() is absolute 1e-6. Build two dists whose value/prob pairs
    // are identical but whose min/max differ by 1e-3 (above 1e-6).
    std::vector<Value_Proba> v4 = {Value_Proba(5.0, 0.5),
                                   Value_Proba(15.0, 0.5)};
    FiniteDist d4(v4);
    FiniteDist d4b(v4);
    d4b.min_time = 5.0 + 1e-3;
    d4b.max_time = 15.0 + 1e-3;
    // 1e-3 on 5.0 is 0.02% relative -> accepted at loose 1e-1, rejected at 1e-9.
    EXPECT_TRUE(d4.approx_equal(d4b, 1e-1));
    EXPECT_FALSE(d4.approx_equal(d4b, 1e-9));
}
TEST(FiniteDist, V1) {
    GaussianDist gau_dis(10, 1);
    FiniteDist finite_dis(gau_dis, 5, 15, 11);
    EXPECT_EQ(11, finite_dis.size());
    EXPECT_EQ(5, finite_dis[0].value);
    EXPECT_EQ(5, finite_dis.min_time);
    EXPECT_EQ(15, finite_dis.max_time);
    EXPECT_NEAR(2.866e-7, finite_dis[0].probability, 1e-8);

    EXPECT_EQ(6, finite_dis[1].value);
    EXPECT_NEAR(3.167e-5 - 2.866e-7, finite_dis[1].probability, 1e-6);
}
TEST(CompressDistributionVector, V1) {
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(3, 0.4),   Value_Proba(4, 0.4),   Value_Proba(5, 0.19),
        Value_Proba(6, 0.001), Value_Proba(7, 0.008), Value_Proba(8, 0.001)};
    CompressDistributionVector(dist_vec1, 3, 5, 1);
    EXPECT_EQ(4, dist_vec1.size());
    EXPECT_EQ(8, dist_vec1[3].value);
    EXPECT_NEAR(0.01, dist_vec1[3].probability, 1e-6);
}
TEST(CompressDistributionVector, V2) {
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(3, 0.4),   Value_Proba(4, 0.4),   Value_Proba(5, 0.19),
        Value_Proba(6, 0.001), Value_Proba(7, 0.008), Value_Proba(8, 0.001)};
    CompressDistributionVector(dist_vec1, 3, 5, 2);
    EXPECT_EQ(5, dist_vec1.size());
    EXPECT_EQ(7, dist_vec1[3].value);
    EXPECT_NEAR(0.009, dist_vec1[3].probability, 1e-6);
    EXPECT_EQ(8, dist_vec1[4].value);
    EXPECT_NEAR(0.001, dist_vec1[4].probability, 1e-6);
}

TEST(FiniteDist, CompressDistribution_v2) {
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(3, 0.1), Value_Proba(4, 0.2), Value_Proba(5, 0.2),
        Value_Proba(6, 0.2), Value_Proba(7, 0.3),
    };
    FiniteDist dist1(dist_vec1);
    dist1.CompressDistribution(2, 0.5);
    // Single-pass conservative: threshold = max(0.5, 0.5) = 0.5.
    // Bucket 1: [3,4,5] → max value 5, prob = 0.5.
    // Bucket 2: [6,7]   → max value 7, prob = 0.5.
    EXPECT_EQ(2, dist1.size());
    EXPECT_NEAR(5.0, dist1.distribution[0].value, 1e-6);
    EXPECT_NEAR(0.5, dist1.distribution[0].probability, 1e-6);
    EXPECT_NEAR(7.0, dist1.distribution[1].value, 1e-6);
    EXPECT_NEAR(0.5, dist1.distribution[1].probability, 1e-6);
}
TEST(FiniteDist, CompressDistribution) {
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(3, 0.1), Value_Proba(4, 0.2), Value_Proba(5, 0.2),
        Value_Proba(6, 0.2), Value_Proba(7, 0.3),
    };
    FiniteDist dist1(dist_vec1);
    dist1.CompressDistribution(2, 0.5);
    // Single-pass buffer-based compression: threshold = 0.5.
    // Elements are accumulated until buf_prob >= threshold, then
    // emitted with the max (conservative) value. This naturally yields at
    // most max_size elements because total probability is 1.0.
    EXPECT_LE(dist1.size(), 2);
}
TEST(FiniteDist, Coalesce_v1) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(0, 0.9),
                                          Value_Proba(4, 0.1)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);
    dist1.Coalesce(dist2);
    EXPECT_EQ(4, dist1.size());
    EXPECT_EQ(0, dist1[0].value);
    EXPECT_EQ(0.9, dist1[0].probability);

    EXPECT_EQ(3, dist1[1].value);
    EXPECT_EQ(0.1, dist1[1].probability);

    EXPECT_EQ(4, dist1[2].value);
    EXPECT_EQ(0.1, dist1[2].probability);

    EXPECT_EQ(7, dist1[3].value);
    EXPECT_EQ(0.9, dist1[3].probability);
}

TEST(FiniteDist, Coalesce_v2) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(5, 0.18),
                                          Value_Proba(8, 0.02)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(5, 0.72),
                                          Value_Proba(6, 0.08)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);
    dist1.Coalesce(dist2);
    EXPECT_EQ(3, dist1.size());
    EXPECT_EQ(5, dist1[0].value);
    EXPECT_NEAR(0.9, dist1[0].probability, 1e-3);

    EXPECT_EQ(6, dist1[1].value);
    EXPECT_EQ(0.08, dist1[1].probability);

    EXPECT_EQ(8, dist1[2].value);
    EXPECT_EQ(0.02, dist1[2].probability);
}

TEST(FiniteDist, Convolve) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(0, 0.9),
                                          Value_Proba(4, 0.1)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);
    dist1.Convolve(dist2);
    EXPECT_EQ(3, dist1.size());
    EXPECT_EQ(3, dist1[0].value);
    EXPECT_NEAR(0.09, dist1[0].probability, 1e-6);

    EXPECT_EQ(7, dist1[1].value);
    EXPECT_NEAR(0.82, dist1[1].probability, 1e-6);

    EXPECT_EQ(11, dist1[2].value);
    EXPECT_NEAR(0.09, dist1[2].probability, 1e-6);
}

TEST(FiniteDist, Convolve_EmptyFirst) {
    FiniteDist empty;
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(0, 0.9),
                                          Value_Proba(4, 0.1)};
    FiniteDist dist2(dist_vec2);
    empty.Convolve(dist2);
    EXPECT_EQ(0, empty.size());
}

TEST(FiniteDist, Convolve_EmptySecond) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};
    FiniteDist dist1(dist_vec1);
    FiniteDist empty;
    dist1.Convolve(empty);
    EXPECT_EQ(0, dist1.size());
}

TEST(FiniteDist, Convolve_CoalesceMultiple) {
    // Many (value, prob) pairs produce identical sums; verify exact coalesce.
    // dist1: [1@0.5, 2@0.5]; dist2: [3@0.5, 4@0.5]
    // Sums: 1+3=4, 1+4=5, 2+3=5, 2+4=6
    // 5 appears twice → must coalesce into single entry with prob=0.5.
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(1, 0.5), Value_Proba(2, 0.5)};
    std::vector<Value_Proba> dist_vec2 = {
        Value_Proba(3, 0.5), Value_Proba(4, 0.5)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);
    dist1.Convolve(dist2);

    EXPECT_EQ(3, dist1.size());

    EXPECT_EQ(4, dist1[0].value);
    EXPECT_NEAR(0.25, dist1[0].probability, 1e-9);

    EXPECT_EQ(5, dist1[1].value);
    EXPECT_NEAR(0.50, dist1[1].probability, 1e-9);

    EXPECT_EQ(6, dist1[2].value);
    EXPECT_NEAR(0.25, dist1[2].probability, 1e-9);
}

TEST(FiniteDist, GetTailDistribution) {
    std::vector<Value_Proba> dist_vec1 = {Value_Proba(3, 0.1),
                                          Value_Proba(7, 0.9)};
    FiniteDist dist1(dist_vec1);

    std::vector<Value_Proba> tail = dist1.GetTailDistribution(5);
    EXPECT_EQ(1, tail.size());
    EXPECT_EQ(7, tail[0].value);

    tail = dist1.GetTailDistribution(8);
    EXPECT_EQ(0, tail.size());

    tail = dist1.GetTailDistribution(3);
    EXPECT_EQ(1, tail.size());
    EXPECT_EQ(7, tail[0].value);
    EXPECT_EQ(0.9, tail[0].probability);
}

TEST(FiniteDist, AddOnePreemption) {
    // this is hp
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                          Value_Proba(5, 0.3)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);

    dist2.AddOnePreemption(dist1, 0);
    EXPECT_EQ(4, dist2.size());

    EXPECT_EQ(5, dist2[0].value);
    EXPECT_NEAR(0.42, dist2[0].probability, 1e-6);
    EXPECT_EQ(6, dist2[1].value);
    EXPECT_NEAR(0.39, dist2[1].probability, 1e-6);
    EXPECT_EQ(7, dist2[2].value);
    EXPECT_NEAR(0.16, dist2[2].probability, 1e-6);
    EXPECT_EQ(8, dist2[3].value);
    EXPECT_NEAR(0.03, dist2[3].probability, 1e-6);
    EXPECT_EQ(5, dist2.min_time);
    EXPECT_EQ(8, dist2.max_time);
}

TEST(FiniteDist, AddOnePreemption_v2) {
    // this is hp
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                          Value_Proba(5, 0.3)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);

    dist2.AddOnePreemption(dist1, 0);
    dist2.AddOnePreemption(dist1, 5);
    EXPECT_EQ(6, dist2.size());

    EXPECT_EQ(5, dist2[0].value);
    EXPECT_NEAR(0.42, dist2[0].probability, 1e-6);
    EXPECT_EQ(7, dist2[1].value);
    EXPECT_NEAR(0.234, dist2[1].probability, 1e-6);
    EXPECT_EQ(8, dist2[2].value);
    EXPECT_NEAR(0.213, dist2[2].probability, 1e-6);
    EXPECT_EQ(9, dist2[3].value);
    EXPECT_NEAR(0.105, dist2[3].probability, 1e-6);
    EXPECT_EQ(10, dist2[4].value);
    EXPECT_NEAR(0.025, dist2[4].probability, 1e-6);
    EXPECT_EQ(11, dist2[5].value);
    EXPECT_NEAR(0.003, dist2[5].probability, 1e-6);
    EXPECT_EQ(5, dist2.min_time);
    EXPECT_EQ(11, dist2.max_time);
}

TEST(FiniteDist, AddPreemption) {
    GlobalVariables::Granularity = 10;
    // this is hp
    std::vector<Value_Proba> dist_vec1 = {
        Value_Proba(1, 0.6), Value_Proba(2, 0.3), Value_Proba(3, 0.1)};
    std::vector<Value_Proba> dist_vec2 = {Value_Proba(4, 0.7),
                                          Value_Proba(5, 0.3)};
    FiniteDist dist1(dist_vec1);
    FiniteDist dist2(dist_vec2);

    dist2.AddPreemption(dist1, 5, 12);
    EXPECT_EQ(7, dist2.size());

    EXPECT_EQ(5, dist2[0].value);
    EXPECT_NEAR(0.42, dist2[0].probability, 1e-6);
    EXPECT_EQ(7, dist2[1].value);
    EXPECT_NEAR(0.234, dist2[1].probability, 1e-6);
    EXPECT_EQ(8, dist2[2].value);
    EXPECT_NEAR(0.213, dist2[2].probability, 1e-6);
    EXPECT_EQ(9, dist2[3].value);
    EXPECT_NEAR(0.105, dist2[3].probability, 1e-6);
    EXPECT_EQ(10, dist2[4].value);
    EXPECT_NEAR(0.025, dist2[4].probability, 1e-6);
    EXPECT_EQ(12, dist2[5].value);
    EXPECT_NEAR(0.0018, dist2[5].probability, 1e-6);

    EXPECT_THAT(12, testing::Le(dist2[6].value));
    EXPECT_NEAR(0.0012, dist2[6].probability, 1e-6);
}

TEST(FiniteDist, constructor) {
    GaussianDist gau_dis(10, 1);
    FiniteDist finite_dist(gau_dis, 10);
    EXPECT_THAT(finite_dist.min_time, testing::Le(10 - 1));
    EXPECT_THAT(finite_dist.max_time, testing::Ge(10 + 1));
}

TEST(FiniteDist, constructor_v2) {
    vector<double> seqs = {1e11, 1e11, 6062, 1e11, 1e11};
    FiniteDist dists(seqs, 10);
    EXPECT_THAT(2, dists.size());
    EXPECT_NEAR(1.0, dists[0].probability + dists[1].probability, 1e-3);
    EXPECT_NEAR(0.2, dists[0].probability, 1e-3);
    EXPECT_NEAR(1e11, dists[1].value, 1e0);
}
TEST(FiniteDist, constructor_v3) {
    vector<double> seqs = {1e3, 1.1e3, 1.2e3, 1.3e3, 2e3,
                           8e3, 11e3,  1e5,   1e11,  1e11};  // 10 elements
    FiniteDist dists(seqs, 4);
    EXPECT_NEAR(0.4, dists.CDF(2e3), 1e-3);
    EXPECT_NEAR(0.6, dists.CDF(1e4), 1e-3);
    EXPECT_NEAR(0.8, dists.CDF(1e5), 1e-3);

    EXPECT_NEAR(1e11, dists[dists.size() - 1].value, 1e0);
    EXPECT_NEAR(0.2, dists[dists.size() - 1].probability, 1e-3);
}

TEST(FiniteDist, AnalyzeFiniteDist_v2) {
    std::vector<double> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    FiniteDist finite_dist(data, 5);
    // EXPECT_EQ(5, finite_dist.size());

    // EXPECT_EQ(1, finite_dist[0].value);
    // EXPECT_EQ(0.1, finite_dist[0].probability);

    // EXPECT_EQ(3.25, finite_dist[1].value);
    // EXPECT_EQ(0.2, finite_dist[1].probability);

    // EXPECT_EQ(5.5, finite_dist[2].value);
    // EXPECT_EQ(0.2, finite_dist[2].probability);

    // EXPECT_EQ(7.75, finite_dist[3].value);
    // EXPECT_EQ(0.2, finite_dist[3].probability);

    // EXPECT_EQ(10, finite_dist[4].value);
    // EXPECT_EQ(0.3, finite_dist[4].probability);

    EXPECT_NEAR(0.1, finite_dist.CDF(1), 1e-3);
    EXPECT_NEAR(0.3, finite_dist.CDF(3.25), 1e-3);
    EXPECT_NEAR(0.5, finite_dist.CDF(5.5), 1e-3);
    EXPECT_NEAR(0.7, finite_dist.CDF(7.75), 1e-3);
}

TEST(FiniteDist, BlockCompress_UnimodalTail) {
    // Test Case 1: Unimodal (Gaussian-like) Tail Compression
    // Single-pass conservative: threshold = max(0.01, 1/6) = 0.1667.
    std::vector<Value_Proba> dist_vec = {
        Value_Proba(0, 0.001),
        Value_Proba(1, 0.005),
        Value_Proba(2, 0.40),
        Value_Proba(3, 0.50),
        Value_Proba(4, 0.08),
        Value_Proba(5, 0.01),
        Value_Proba(6, 0.002),
        Value_Proba(7, 0.002)
    };
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(6, 1.0 / 6.0);

    EXPECT_EQ(2, dist.size());
    EXPECT_NEAR(2.0, dist.distribution[0].value, 1e-4);
    EXPECT_NEAR(0.406, dist.distribution[0].probability, 1e-4);
    EXPECT_NEAR(7.0, dist.distribution[1].value, 1e-4);
    EXPECT_NEAR(0.594, dist.distribution[1].probability, 1e-4);
}

TEST(FiniteDist, BlockCompress_MultimodalValley) {
    // Test Case 2: Multimodal (Two Peaks) Valley Compression
    // Single-pass conservative: threshold = 1.0/5 = 0.2.
    std::vector<Value_Proba> dist_vec = {
        Value_Proba(0, 0.001),
        Value_Proba(1, 0.55),
        Value_Proba(2, 0.002),
        Value_Proba(3, 0.001),
        Value_Proba(4, 0.445),
        Value_Proba(5, 0.001)
    };
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(5, 0.2);

    EXPECT_EQ(2, dist.size());
    EXPECT_NEAR(1.0, dist.distribution[0].value, 1e-4);
    EXPECT_NEAR(0.551, dist.distribution[0].probability, 1e-4);
    EXPECT_NEAR(5.0, dist.distribution[1].value, 1e-4);
    EXPECT_NEAR(0.449, dist.distribution[1].probability, 1e-4);
}

TEST(FiniteDist, BlockCompress_FlatDistribution) {
    // Uniform distribution compressed with threshold = 1.0/10 = 0.1.
    // All elements have equal prob 1/15 ≈ 0.067. Pairs cross threshold,
    // yielding buckets of ~0.133 each; trailing element merges into last.
    std::vector<Value_Proba> dist_vec;
    for (int i = 0; i < 15; ++i) {
        dist_vec.push_back(Value_Proba(i, 1.0 / 15.0));
    }
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(10, 0.1);

    EXPECT_LE(dist.size(), 10);
}

TEST(FiniteDist, BlockCompress_RegressionMonotonicBug) {
    // Test Case 4: Regression test — old monotonic code would compress from index 0
    // to end, destroying the dual-peak structure.
    // Single-pass conservative preserves peak separation by using max values.
    std::vector<Value_Proba> dist_vec = {
        Value_Proba(0, 0.001),
        Value_Proba(1, 0.55),
        Value_Proba(2, 0.002),
        Value_Proba(3, 0.001),
        Value_Proba(4, 0.445),
        Value_Proba(5, 0.001)
    };
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(5, 0.2);

    EXPECT_EQ(2, dist.size());
    EXPECT_NEAR(1.0, dist.distribution[0].value, 1e-4);
    EXPECT_NEAR(0.551, dist.distribution[0].probability, 1e-4);
    EXPECT_NEAR(5.0, dist.distribution[1].value, 1e-4);
    EXPECT_NEAR(0.449, dist.distribution[1].probability, 1e-4);
}

TEST(FiniteDist, BlockCompress_AlreadySmall) {
    // All elements above threshold, size already <= max_size → early exit.
    std::vector<Value_Proba> dist_vec = {
        Value_Proba(0, 0.2),
        Value_Proba(1, 0.3),
        Value_Proba(2, 0.5)
    };
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(10, 0.1);

    EXPECT_EQ(3, dist.size());
    EXPECT_EQ(0, dist.distribution[0].value);
    EXPECT_EQ(1, dist.distribution[1].value);
    EXPECT_EQ(2, dist.distribution[2].value);
}

TEST(FiniteDist, BlockCompress_AlternatingNoise) {
    // High-frequency noise: threshold = 1.0/6 ≈ 0.167.
    // [0.009],[0.5]       → bucket 1: 0.509 >= 0.167, emit (1, 0.509)
    // [0.009],[0.5]       → bucket 2: 0.509 >= 0.167, emit (3, 0.509)
    // [0.009],[0.5]       → bucket 3: 0.509 >= 0.167, emit (5, 0.509)
    // trailing [0.009]    → merge into last: emit (7, 0.518)
    std::vector<Value_Proba> dist_vec = {
        Value_Proba(0, 0.009),
        Value_Proba(1, 0.5),
        Value_Proba(2, 0.009),
        Value_Proba(3, 0.5),
        Value_Proba(4, 0.009),
        Value_Proba(5, 0.5),
        Value_Proba(6, 0.009),
        Value_Proba(7, 0.5)
    };
    FiniteDist dist(dist_vec);
    dist.CompressDistribution(6, 1.0 / 6.0);

    EXPECT_LE(dist.size(), 6);
}

TEST(FiniteDist, BlockCompress_EmptyAndSingleElement) {
    // Edge cases: empty and single-element distributions.
    // Early-exits before threshold check (size <= max_size).
    std::vector<Value_Proba> empty_vec;
    FiniteDist dist_empty(empty_vec);
    dist_empty.CompressDistribution(10, 0.125);
    EXPECT_EQ(0, dist_empty.size());

    std::vector<Value_Proba> single_vec = {Value_Proba(5, 1.0)};
    FiniteDist dist_single(single_vec);
    dist_single.CompressDistribution(10, 0.125);
    EXPECT_EQ(1, dist_single.size());
    EXPECT_EQ(5, dist_single.distribution[0].value);
}

// ============================================================================
// Idea 10: Single-Point (Degenerate) Convolution Fast Path
// ============================================================================
// Convolve(A, {(v, 1.0)}) is mathematically a pure value-shift:
//   {(a.value + v, a.probability) for a in A}
// A uniform shift preserves sorted order and creates no duplicate values, so
// no sort and no coalesce are needed. The fast path in Probability.cpp exploits
// this. These tests pin the contract so the fast path cannot drift from the
// general N×M + sort + coalesce path. They are characterization tests: they
// pass today (slow path) and must keep passing after the fast path lands.
// Single-point operands arise from GetUnitExecutionTimeDist(time_limit)
// (Probability.h), which replaces a TL'd perf-pair task's ET with a degenerate
// distribution once a time limit is applied.

TEST(FiniteDist, Convolve_SinglePointOther_ShiftsValues) {
    // A (multi-point) convolved with single-point B(v=5):
    // each A value += 5, probabilities unchanged, sorted order preserved.
    std::vector<Value_Proba> a_vec = {Value_Proba(1, 0.2), Value_Proba(4, 0.3),
                                      Value_Proba(9, 0.5)};
    FiniteDist a(a_vec);
    FiniteDist b = GetUnitExecutionTimeDist(5.0);  // single-point {(5, 1.0)}
    a.Convolve(b);
    EXPECT_EQ(3u, a.size());
    EXPECT_EQ(6, a[0].value);
    EXPECT_NEAR(0.2, a[0].probability, 1e-9);
    EXPECT_EQ(9, a[1].value);
    EXPECT_NEAR(0.3, a[1].probability, 1e-9);
    EXPECT_EQ(14, a[2].value);
    EXPECT_NEAR(0.5, a[2].probability, 1e-9);
    EXPECT_EQ(6, a.min_time);
    EXPECT_EQ(14, a.max_time);
}

TEST(FiniteDist, Convolve_SinglePointSelf_ShiftsOther) {
    // Symmetric: single-point A(v=5) convolved with multi-point B.
    // Result is B's distribution shifted by +5 (order follows B's sorted order).
    // This pins that the size-1-`this` branch rebuilds `this` from `other`.
    std::vector<Value_Proba> b_vec = {Value_Proba(2, 0.4), Value_Proba(7, 0.6)};
    FiniteDist a = GetUnitExecutionTimeDist(5.0);
    FiniteDist b(b_vec);
    a.Convolve(b);
    EXPECT_EQ(2u, a.size());
    EXPECT_EQ(7, a[0].value);
    EXPECT_NEAR(0.4, a[0].probability, 1e-9);
    EXPECT_EQ(12, a[1].value);
    EXPECT_NEAR(0.6, a[1].probability, 1e-9);
    EXPECT_EQ(7, a.min_time);
    EXPECT_EQ(12, a.max_time);
}

TEST(FiniteDist, Convolve_SinglePointZero_IsIdentity) {
    // Convolve with single-point at 0 is identity (no shift).
    std::vector<Value_Proba> a_vec = {Value_Proba(3, 0.1), Value_Proba(7, 0.9)};
    FiniteDist a(a_vec);
    FiniteDist zero = GetUnitExecutionTimeDist(0.0);
    a.Convolve(zero);
    EXPECT_EQ(2u, a.size());
    EXPECT_EQ(3, a[0].value);
    EXPECT_NEAR(0.1, a[0].probability, 1e-9);
    EXPECT_EQ(7, a[1].value);
    EXPECT_NEAR(0.9, a[1].probability, 1e-9);
    EXPECT_EQ(3, a.min_time);
    EXPECT_EQ(7, a.max_time);
}

TEST(FiniteDist, Convolve_SinglePointScalesProbability) {
    // A single-point operand need NOT have probability 1.0. The degenerate
    // convolution Convolve(A, {(v,p)}) = {(a.value+v, a.probability*p)} —
    // values shift by v AND probabilities scale by p. This is the case that
    // actually fires in AddOnePreemption (the preemption tail is a single
    // mass point with p<1). Pinned so the fast path never drops the scale.
    std::vector<Value_Proba> a_vec = {Value_Proba(4, 0.7), Value_Proba(5, 0.3)};
    FiniteDist a(a_vec);
    // single-point {(11, 0.003)} — NOT produced by GetUnitExecutionTimeDist,
    // but Convolve is a general API and must honor it.
    FiniteDist sp(std::vector<Value_Proba>{Value_Proba(11, 0.003)});
    a.Convolve(sp);
    EXPECT_EQ(2u, a.size());
    EXPECT_EQ(15, a[0].value);
    EXPECT_NEAR(0.7 * 0.003, a[0].probability, 1e-9);
    EXPECT_EQ(16, a[1].value);
    EXPECT_NEAR(0.3 * 0.003, a[1].probability, 1e-9);
    EXPECT_EQ(15, a.min_time);
    EXPECT_EQ(16, a.max_time);
}

TEST(FiniteDist, Convolve_SinglePointCoalescesDuplicateValues) {
    // When the multi-point operand has duplicate values, a uniform shift keeps
    // those duplicates adjacent; the fast path must coalesce them to match the
    // general N×M + sort + coalesce path exactly. Construct A with two equal
    // values; after shift they must merge into one entry summing probabilities.
    std::vector<Value_Proba> a_vec = {Value_Proba(2, 0.4), Value_Proba(2, 0.1),
                                      Value_Proba(5, 0.5)};
    FiniteDist a(a_vec);
    FiniteDist sp = GetUnitExecutionTimeDist(3.0);  // shift by +3
    a.Convolve(sp);
    // {2,2,5} shifted by +3 -> {5,5,8}; the two 5s coalesce -> {5@0.5, 8@0.5}
    EXPECT_EQ(2u, a.size());
    EXPECT_EQ(5, a[0].value);
    EXPECT_NEAR(0.5, a[0].probability, 1e-9);
    EXPECT_EQ(8, a[1].value);
    EXPECT_NEAR(0.5, a[1].probability, 1e-9);
    EXPECT_EQ(5, a.min_time);
    EXPECT_EQ(8, a.max_time);
}

TEST(FiniteDist, Convolve_SinglePointPreservesOrderAndProbs) {
    // Larger, non-uniformly-spaced distribution with a NEGATIVE shift.
    // Shift must keep it sorted with probabilities intact (exercises both
    // shift directions and confirms min_time/max_time track the shift).
    std::vector<Value_Proba> a_vec = {Value_Proba(-2, 0.05), Value_Proba(0, 0.15),
                                      Value_Proba(3, 0.30),
                                      Value_Proba(7, 0.50)};
    FiniteDist a(a_vec);
    FiniteDist shift = GetUnitExecutionTimeDist(-2.5);
    a.Convolve(shift);
    EXPECT_EQ(4u, a.size());
    // sorted order preserved (uniform shift of a sorted vector stays sorted)
    for (size_t i = 1; i < a.size(); ++i)
        EXPECT_GT(a[i].value, a[i - 1].value);
    EXPECT_EQ(-4.5, a[0].value);
    EXPECT_NEAR(0.05, a[0].probability, 1e-9);
    EXPECT_EQ(-2.5, a[1].value);
    EXPECT_NEAR(0.15, a[1].probability, 1e-9);
    EXPECT_EQ(0.5, a[2].value);
    EXPECT_NEAR(0.30, a[2].probability, 1e-9);
    EXPECT_EQ(4.5, a[3].value);
    EXPECT_NEAR(0.50, a[3].probability, 1e-9);
    EXPECT_EQ(-4.5, a.min_time);
    EXPECT_EQ(4.5, a.max_time);
}

// Differential test: across a table of representative multi-point distributions
// and shift values, Convolve(A, single-point(v, p)) must equal an independent
// hand-rolled reference (values += v, probabilities *= p, then sort + coalesce).
// This is the oracle that would catch a fast-path bug such as forgetting to
// update min_time/max_time, dropping the probability scale, or skipping the
// adjacent coalesce on duplicate values.
TEST(FiniteDist, Convolve_SinglePointMatchesShiftReference) {
    struct Case {
        std::vector<Value_Proba> a;
        double shift;
        double prob;  // probability mass of the single-point operand
    };
    std::vector<Case> cases = {
        // p = 1.0 (the GetUnitExecutionTimeDist case): pure shift.
        {{Value_Proba(1, 0.5), Value_Proba(2, 0.5)}, 10.0, 1.0},
        {{Value_Proba(0, 0.2), Value_Proba(5, 0.3), Value_Proba(100, 0.5)}, 7.5,
         1.0},
        {{Value_Proba(-10, 0.25), Value_Proba(0, 0.25), Value_Proba(10, 0.25),
          Value_Proba(20, 0.25)}, -3.0, 1.0},
        // p < 1.0 (the AddOnePreemption tail case): shift AND scale.
        {{Value_Proba(4, 0.7), Value_Proba(5, 0.3)}, 11.0, 0.003},
        // Duplicate values in A that coalesce after shift.
        {{Value_Proba(2, 0.4), Value_Proba(2, 0.1), Value_Proba(5, 0.5)}, 3.0,
         1.0},
    };
    for (const Case& c : cases) {
        FiniteDist a(c.a);
        FiniteDist sp(std::vector<Value_Proba>{Value_Proba(c.shift, c.prob)});
        a.Convolve(sp);

        // Independent reference: shift values, scale probs, sort, coalesce.
        std::vector<Value_Proba> ref = c.a;
        for (auto& vp : ref) {
            vp.value += c.shift;
            vp.probability *= c.prob;
        }
        std::sort(ref.begin(), ref.end(),
                  [](const Value_Proba& x, const Value_Proba& y) {
                      return x.value < y.value;
                  });
        std::vector<Value_Proba> ref_merged;
        ref_merged.reserve(ref.size());
        for (const auto& item : ref) {
            if (!ref_merged.empty() &&
                ref_merged.back().value == item.value) {
                ref_merged.back().probability += item.probability;
            } else {
                ref_merged.push_back(item);
            }
        }
        FiniteDist ref_dist(ref_merged);

        EXPECT_EQ(ref_merged.size(), a.size());
        EXPECT_TRUE(a.approx_equal(ref_dist, 1e-9))
            << "shift=" << c.shift << " prob=" << c.prob;
    }
}

int main(int argc, char** argv) {
    // ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}