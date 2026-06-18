#ifndef TESTFRAMEWORK_H
#define TESTFRAMEWORK_H

#include <iostream>
#include <string>
#include <cmath>
#include <functional>
#include <vector>

/**
 * Minimal, self-contained unit test "framework" (no external library).
 * Each test prints its name, the input it was given, the expected vs
 * actual output, and a PASS/FAIL verdict directly to the console.
 */
namespace TestFramework {

inline int g_passed = 0;
inline int g_failed = 0;

inline void reportResult(const std::string& testName, bool passed,
                          const std::string& details) {
    std::cout << "[" << (passed ? "PASS" : "FAIL") << "] " << testName << "\n";
    std::cout << details;
    std::cout << "------------------------------------------------------------\n";
    if (passed) g_passed++; else g_failed++;
}

inline bool nearlyEqual(double a, double b, double eps = 1e-6) {
    if (std::isinf(a) && std::isinf(b)) return true;
    return std::fabs(a - b) <= eps;
}

inline void printSummary() {
    std::cout << "\n============================================================\n";
    std::cout << "TEST SUMMARY: " << g_passed << " passed, " << g_failed << " failed, "
              << (g_passed + g_failed) << " total\n";
    std::cout << "============================================================\n";
}

} // namespace TestFramework

#endif
