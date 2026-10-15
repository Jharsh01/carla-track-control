// Minimal checks for the tests: no test framework needed.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

inline int check_failures = 0;

#define CHECK(condition)                                                                 \
    do {                                                                                 \
        if (!(condition)) {                                                              \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            ++check_failures;                                                            \
        }                                                                                \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        const double check_a_ = (a), check_b_ = (b);                                                   \
        if (!(std::fabs(check_a_ - check_b_) <= (tol))) {                                              \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.9g, %s = %.9g, tolerance %g\n", __FILE__, \
                         __LINE__, #a, check_a_, #b, check_b_, static_cast<double>(tol));             \
            ++check_failures;                                                                          \
        }                                                                                              \
    } while (0)

inline int check_report(const char* name) {
    if (check_failures) std::fprintf(stderr, "%s: %d check(s) failed\n", name, check_failures);
    else std::printf("%s: all checks passed\n", name);
    return check_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
